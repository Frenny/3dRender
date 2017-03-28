package sim;

import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import javax.swing.JFrame;
import javax.swing.JTextField;
import javax.swing.JTextArea;
import javax.swing.JLabel;
import javax.swing.JLayeredPane;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JScrollPane;
import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.SwingConstants;
import javax.swing.JOptionPane;

import java.awt.event.WindowListener;
import java.awt.event.WindowEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ActionEvent;
import javax.swing.event.ChangeListener;

import javafx.application.Platform;
import javafx.scene.shape.Shape3D;

import javax.swing.event.ChangeEvent;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Image;
import java.awt.Toolkit;

import java.util.Locale;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.FieldPosition;
import java.text.ParseException;

import lejos.pc.comm.NXTConnector;
import lejos.nxt.LCD;
import jr.motion.DCMotorController;
import tetrix.Tetrix;


import java.io.File;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.IOException;
import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.net.Socket;
import java.net.InetAddress;


import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.mss.MotionSubsystem;
import robotinterface.mss.MotionSubsystemListener;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.util.StringUtil;

import sim.environment.Environment;
import sim.keypoints.KeyPointRecognizer;
import sim.util.SimDebugPainter;
import sim.util.SimDebugConsole;
import sim.util.RemoteDebugClient;
import sim.util.DebugPainterOverlaySelectionDialog;
import sim.util.PositionTrigger;
import sim.util.PositionListener;
import sim.util.ComplexPositionTrigger;
import sim.util.ComplexPositionListener;
import sim.util.MSSEstimator;
import sim.camera3d.CameraLocation;
import sim.camera3d.CameraSize;
import sim.camera3d.Environment3D;
import sim.camera3d.objects.Wall;
import sim.camera3d.Camera3D;
import sim.customsim.Customize2DKeypoints;
import sim.customsim.Customize3DKeypoints;


import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import java.awt.event.ComponentListener;

/**
* Main class of the Carbot simulator.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class CarbotSim extends JFrame implements ActionListener,ChangeListener,WindowListener,MotionSubsystem {

    private JTextField commandField;
    private JTextArea resultField;
    private JTextArea asyncField;
    private JTextArea lcdField;
    private JTextArea posField;
    private JTextField collisionField;

    private JButton zoomInButton; 
    private JButton zoomOutButton;

    private JButton resetButton; 
    private JButton startButton; 
    private JButton pauseButton;
    private JButton stopButton;


    private JCheckBox soundCheck;
    private JComboBox<String> volCombo;

    private JCheckBox consoleCheck;
    private JCheckBox followCheck;

    private JComboBox<String> camLookCombo;
    private JComboBox<String> camLocCombo;
    private JComboBox<String> camSizeCombo;

    private JButton debugPaintSelButton; 


    private JCheckBox showGridCheck;
    private JCheckBox showInternalPoseCheck;
    private JCheckBox showKeypointsCheck;
    private JCheckBox showCamRangeCheck;
    private JCheckBox showShaddowCheck;
    private JCheckBox showSlickCheck;
    private JCheckBox showCollisionsCheck;
    private JCheckBox showUSCheck;
    private JCheckBox showTrackCheck;
    private JCheckBox showTyreTracksCheck;
    private JButton clearTrackButton; 


    private JLayeredPane canvasPane;
    private SimCanvas simCanvas;
    private Camera3D camera3D;

//    private final static long LIFE_SIGN_CYCLE=5000;

    private final static String SCREENCONFIGFILE="carbotsim.cfg";


    private boolean initialTestCommand=false;
    private boolean initialTestSuccessful=false;


    private static String protocol="usb";
    private static String nxtName=null;

    private NXTConnector conn=null;     // Connector zum NXT-Baustein (Motion-Subsystem)
    private DataOutputStream dos=null;  // Output zum Motion-Subsystem (Commandos)
    private DataInputStream dis=null;   // Input from Motion-Subsystem (Antworten sync & async)


    private Environment environment=null;
    private static String environmentFile=null;


    public static double asyncValueX=Double.NaN;
    public static double asyncValueY=Double.NaN;
    public static double asyncValueANGLE=Double.NaN;
    public static double sinAsyncValueANGLE=Double.NaN;
    public static double cosAsyncValueANGLE=Double.NaN;

    public static KeyPointRecognizer keyPointRecognizer=null;

    public static MSSEstimator mssEstimator=null;


    private MotionSubsystemListener motionSubsystemListener=null;
    private Class controllerClass=null;
    private RobotController controllerInstance=null;


    private long lastAsyncMessageTime=-1;  // Zeitpunkt des letzten Emfangs einer Asynchronen Positionsmeldung (regul�r UND bei CommandStart/Stop)
    private long lastAsyncPeriodic=-1;     // Zeitspanne, die durch "periodic" gesetzt wurde (allerdings gemessen anhand der letzten beiden Async-Messages)



// F�r Remote Debugging
    private boolean remoteDebugging=false;
    private DataOutputStream outputStreamCmd=null;   // Run, Pause, Stop, Command
    private DataInputStream inputStreamCmd=null; // Status�nderung bez�glich Run, Pause, Stop
    private DataInputStream inputStreamMSS=null; // Antworten des Motion-Subsystems (sync & asnc)
    private DataInputStream inputStreamDebug=null; // Mal-Kommandos des Remote Debug-Painters und Remote Debug-Out
    private DataInputStream inputStreamCam=null; // Life-Camera-Bild


    private SimDebugConsole simDebugConsole=null;



    public static void main(String[] args) {

        System.out.println("**************************************");
        System.out.println("* Carbot Simulator by J\u00f6rg Roth");
        System.out.println("* Version:          "+SimCanvas.VERSION_STR);
        System.out.println("* Motion Subsystem: "+Tetrix.VERSION);
        System.out.println("* Robot Interface:  "+Robot.VERSION);
        System.out.println("**************************************");
        System.out.println("");

        int optNr=0;
        String controllerClassname=null;
        String controllerConfiguration=null;
        String remoteDebuggingHost=null;
        String customize2DkeypointsClassname=null;
        String customize3DkeypointsClassname=null;
        String directiveCAMERA=null;
        String directiveKEYPOINTS=null;

        int vOpt=-1;
        boolean[] hasV=new boolean[2];
        double[] vValue=new double[hasV.length];

        if (args.length>0 && args[0].equals("-?")) {
            System.out.println("CarbotSim - the Carbot simulator by J. Roth");
            System.out.println("");
            System.out.println("Usage CarbotSim <optional params>");
            System.out.println("Where <optional params> are");
            System.out.println("  -e <environment>: text file that defines the environment");
            System.out.println("  -c <controller>: class name of the robot controller that controls the Carbot");
            System.out.println("  -cfg <controller-config>: pass a configuration string to the robto controller");
            System.out.println("  -r <host>: perform remote debugging to real carbot with this host address");
            System.out.println("  -custom2d <classname>: use the class to customize the 2D keypoints");
            System.out.println("  -custom3d <classname>: use the class to customize the 3D keypoints");
            System.out.println("  -camera <cam>: set CAMERA directive (same as in environment file)");
            System.out.println("  -keypoints <kp>: set KEYPOINTS directive (same as in environment file)");
            System.out.println("  -v0, -v1... : set internal (test) values");
            System.exit(1);
        }


        for (int i=0;i<args.length;i++) {
            String opt=args[i].toLowerCase();
            if (opt.equals("-e")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=1;
            }
            else if (opt.equals("-c")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=2;
            }
            else if (opt.equals("-r")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=3;
            }
            else if (opt.equals("-cfg")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=4;
            }
            else if (opt.equals("-custom2d")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=5;
            }
            else if (opt.equals("-custom3d")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=6;
            }            
            else if (opt.equals("-camera")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=7;
            }     
            else if (opt.equals("-keypoints")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=8;
            }
            else if (opt.startsWith("-v")) {
                if (optNr!=0) {
                    System.out.println("Two options without argument (second was "+opt+")");
                    System.exit(1);
                }
                optNr=9;
                try {
                    vOpt=new Integer(opt.substring(2)).intValue();
                }
                catch (NumberFormatException e) {
                    System.out.println("No nr in v option "+opt);
                    System.exit(1);
                }
                if (vOpt<0 || vOpt>=hasV.length) {
                    System.out.println("Nr in v option hat to be 0..."+(hasV.length-1)+" in "+opt);
                    System.exit(1);
                }
            }
            else {
                switch (optNr) {
                    case 0:
                        System.out.println("Argument '"+opt+" 'without option");
                        System.out.println("Use -? for help");
                        System.exit(1);
                        break; 
                    case 1:  // -e
                        if (environmentFile!=null) {
                            System.out.println("Two environments passed");
                            System.exit(1);
                        }
                        environmentFile=args[i];
                        optNr=0;
                        break; 
                    case 2:  // -c
                        if (controllerClassname!=null) {
                            System.out.println("Two controller passed");
                            System.exit(1);
                        }
                        controllerClassname=args[i];
                        optNr=0;
                        break; 
                    case 3:  // -r
                        if (remoteDebuggingHost!=null) {
                            System.out.println("Two hosts passed");
                            System.exit(1);
                        }
                        remoteDebuggingHost=args[i];
                        optNr=0;
                        break; 
                    case 4:  // -cfg
                        if (controllerConfiguration!=null) {
                            System.out.println("Two cfg passed");
                            System.exit(1);
                        }
                        controllerConfiguration=args[i];
                        optNr=0;
                        break;                         

                    case 5:  // -custom2d
                        if (customize2DkeypointsClassname!=null) {
                            System.out.println("Two custom2d passed");
                            System.exit(1);
                        }
                        customize2DkeypointsClassname=args[i];
                        optNr=0;
                        break;  
                    case 6:  // -custom3d
                        if (customize3DkeypointsClassname!=null) {
                            System.out.println("Two custom3d passed");
                            System.exit(1);
                        }
                        customize3DkeypointsClassname=args[i];
                        optNr=0;
                        break;  
                    case 7:  // -CAMERA
                        if (directiveCAMERA!=null) {
                            System.out.println("Two CAMERA passed");
                            System.exit(1);
                        }
                        directiveCAMERA=args[i];
                        optNr=0;
                        break;  
                    case 8:  // -KEYPOINTS
                        if (directiveKEYPOINTS!=null) {
                            System.out.println("Two KEYPOINTS passed");
                            System.exit(1);
                        }
                        directiveKEYPOINTS=args[i];
                        optNr=0;
                        break;  
                    case 9:  // -v<nr>
                        if (hasV[vOpt]) {
                            System.out.println("Two values for v option "+vOpt+" passed");
                            System.exit(1);
                        }
                        try {
                            vValue[vOpt]=new Double(args[i]).doubleValue();
                        }
                        catch (IllegalArgumentException e) {
                            System.out.println("Illegal nr value "+args[i]+" in v option "+vOpt);
                            System.exit(1);
                        }
                        hasV[vOpt]=true;
                        optNr=0;
                        break;  
                }
            }
        }
        if (optNr!=0) {
            System.out.println("Last option without argument");
            System.exit(1);
        }


        if (hasV[0]) {
           if (vValue[0]>0.0d)
               Environment.LINEIDS_NOREFRESH=true;
        }
        if (hasV[1]) {
            if (vValue[1]<0) {
                System.out.println("Value for -v1 must not be less than 0");
                System.exit(1);
            }
            KeyPointRecognizer.KEYPOINT3DVSTDDEV=vValue[1];
        }


        if (remoteDebuggingHost!=null) {   // -r (Remote Debugging)
            if (controllerClassname!=null) {
                System.out.println("Cannot perform remote debugging and use a local controller at the same time");
                System.exit(1);
            }
            if (environmentFile!=null) {
                System.out.println("Remote debugging requires empty environment");
                System.exit(1);
            }
            if (customize2DkeypointsClassname!=null || customize3DkeypointsClassname!=null) {
                System.out.println("Remote debugging prohibits custumization");
                System.exit(1);
            }
        }

        if (controllerConfiguration!=null && controllerClassname==null) {
            System.out.println("Controller configuration requires a controller");
            System.exit(1);
        }

        nxtName="NXT";

        new CarbotSim(environmentFile,controllerClassname,controllerConfiguration,remoteDebuggingHost,customize2DkeypointsClassname,customize3DkeypointsClassname,directiveCAMERA,directiveKEYPOINTS);
    }


    public CarbotSim(String environmentFile,String controllerClassname,String controllerConfiguration,String remoteDebuggingHost,
                     String customize2DkeypointsClassname,String customize3DkeypointsClassname,
                     String directiveCAMERA,String directiveKEYPOINTS) {

        if (remoteDebuggingHost!=null) {
            remoteDebugging=true;
            try {
                waitForRemoteDebuggingHost(remoteDebuggingHost);
                createInputThreads();
            }
            catch (IOException e) {
                System.out.println("Cannot establish remote debugging to host "+remoteDebuggingHost+" ("+e.toString()+")");
                System.exit(1);
            }
        }


   // Wenn gew�nscht Instanz f�r customize2Dkeypoints anlegen
        Customize2DKeypoints customize2DkeypointsInstance=null;
        if (customize2DkeypointsClassname!=null) {
            try {
                Class customize2DkeypointsClass=Class.forName(customize2DkeypointsClassname);
                customize2DkeypointsInstance=(Customize2DKeypoints)customize2DkeypointsClass.newInstance();
                System.out.println("use class "+customize2DkeypointsClassname+" to customize 2D keypoints");
            }
            catch (Exception e) {
                System.out.println("Cannot instantiate customize2Dkeypoints controller class '"+customize2DkeypointsClassname+"': "+e);
                e.printStackTrace();
                System.exit(1);
            }
        }

   // Wenn gew�nscht Instanz f�r customize3Dkeypoints anlegen
        Customize3DKeypoints customize3DkeypointsInstance=null;
        if (customize3DkeypointsClassname!=null) {
            try {
                Class customize3DkeypointsClass=Class.forName(customize3DkeypointsClassname);
                customize3DkeypointsInstance=(Customize3DKeypoints)customize3DkeypointsClass.newInstance();
                System.out.println("use class "+customize3DkeypointsClassname+" to customize 3D keypoints");
            }
            catch (Exception e) {
                System.out.println("Cannot instantiate customize3Dkeypoints controller class '"+customize3DkeypointsClassname+"': "+e);
                e.printStackTrace();
                System.exit(1);
            }
        }



    // Die Subsysteme mit dieser Instanz verbinden
        Robot.motionSubsystem=this;     // Das Motion subsystem wird von dieser Klasse selbst simuliert
 
        keyPointRecognizer=new KeyPointRecognizer(customize2DkeypointsInstance,customize3DkeypointsInstance);   // Das Vision subsystem wird von einem KeyPointRegognizer3D geleistet
        Robot.visionSubsystem=keyPointRecognizer;

        mssEstimator=new MSSEstimator(1000);  // BuffSize 

        PositionTrigger positionTrigger=new PositionTrigger(mssEstimator);                                         // PositionEstimator braucht regelm��ig die Position
        ComplexPositionTrigger complexPositionTrigger=new ComplexPositionTrigger(mssEstimator,keyPointRecognizer); // Optical Flow braucht regelm��ig die Position (auch estimated)
        keyPointRecognizer.setComplexPositionTrigger(complexPositionTrigger);                                      // Setzen, damit man �ber das VSS das Timing �ndern kann
        
   // Instantiiere einen Debug-Painter
        Robot.debugPainter=new SimDebugPainter();

   // Instantiiere Console f�r Debug-Out        
        simDebugConsole=new SimDebugConsole(this);
        Robot.debugOut=simDebugConsole;
                
        if (remoteDebugging) {
            new RemoteDebugClient(inputStreamDebug,Robot.debugPainter,Robot.debugOut);  // Bei Remote-Debugging: �ber das Netzwerk Kommandos einspeisen
        }

        Sound.initSound();

        try {
            environment=new Environment(environmentFile);
            if (environment.robotControllerConfig!=null) {
                if (controllerClassname==null) 
                    System.out.println("Note: Environment file contains a robot controller configuration, but no robot controller was defined - ignore environment config");
                else if (controllerConfiguration!=null)
                    System.out.println("Note: Environment file contains a robot controller configuration, but also the command line - ignore environment config");
                else 
                    controllerConfiguration=environment.robotControllerConfig;
            }

            if (directiveCAMERA!=null) {
                if (environment.hasCamera) {
                    System.out.println("Note: Environment file contains a CAMERA directive configuration, but also the command line - ignore environment config");
                }
                try {
                    directiveCAMERA="CAMERA "+directiveCAMERA;
                    ArrayList<String> args=environment.args(directiveCAMERA);
                    environment.processCAMERAdirective(directiveCAMERA,args);
                }
                catch (Exception e) {
                    System.out.println("Cannot process CAMERA directive: "+e.toString());
                    System.exit(1);
                }
            }

            if (directiveKEYPOINTS!=null) {
                if (environment.hasKeypointsConfig) {
                    System.out.println("Note: Environment file contains a KEYPOINTS directive configuration, but also the command line - ignore environment config");
                }
                try {
                    directiveKEYPOINTS="KEYPOINTS "+directiveKEYPOINTS;
                    ArrayList<String> args=environment.args(directiveKEYPOINTS);
                    environment.processKEYPOINTSdirective(directiveKEYPOINTS,args);
                }
                catch (Exception e) {
                    System.out.println("Cannot process KEYPOINTS directive: "+e.toString());
                    System.exit(1);
                }
            }

        }
        catch (Exception e) {
            System.out.println("Cannot read environment "+e);
            System.exit(1);
        }
        
        
   // Wenn gew�nscht, einen Robot-Controller instantiieren
        if (controllerClassname!=null) {
            try {
                controllerClass=Class.forName(controllerClassname);
                controllerInstance=(RobotController)controllerClass.newInstance();
                System.out.println("Robot controller instantiated:");
                System.out.println("----------------- DESCRIPTION -----------------");
                System.out.println(controllerInstance.getDescription());
                System.out.println("-------------- END OF DESCRIPTION -------------");
                
                if (controllerInstance.requiresConfiguration() && controllerConfiguration==null) {
                    System.out.println("Robot controller requires a configuration, but no was passed (use -cfg to pass one)");
                    System.exit(1);
                }
                if (controllerConfiguration!=null) {
                    try {
                        controllerInstance.configure(controllerConfiguration);
                    }
                    catch (IllegalArgumentException e) {
                        System.out.println("Robot controller does not accept the given configuration '"+controllerConfiguration+"':");
                        System.out.println(e.toString());
                        System.exit(1);                
                    }
                }
            }
            catch (Exception e) {
                System.out.println("Cannot instantiate robot controller class '"+controllerClassname+"': "+e);
                e.printStackTrace();
                System.exit(1);
            }
        }        

        try {
            initUI();

// Wenn Remote-Debugging: kein Aufbau einer Verbindung zum Motion-Subsystem
            if (remoteDebugging) {
                Tetrix.isUp=true;   // So tun, als ob Tetrix l�uft
            }

// Wenn kein Remote-Debugging: Aufbau einer Verbindung zum Motion-Subsystem und starten eines Threads, der die Meldungen einholt
            else {


// Die eigene IP-Adresse nur zu Anzeige-Zwecken
                String ipAddress=getIPAddress();

// Aufbau einer Verbindung zum simulierten Motion-Subsystem via simulierterm USB
                conn=new NXTConnector();
                String connStr=protocol+"://"+nxtName;

                System.out.println("Connection with "+connStr);

                boolean connected=false;

                int tries=100;

                while (tries>0 && !connected) {
                    connected=conn.connectTo(connStr);
                    if (!connected) {
                        System.out.println("Try to connect failed, "+tries+" more tries");
                        tries--;
                        try {Thread.sleep(2000);} catch (Exception e) {}
                    }
                }
		
                if (!connected) {
                    System.err.println("Failed to connect to NXT named "+nxtName);
                    System.exit(1);
                }
		
                dos=new DataOutputStream(conn.getOutputStream());
                dis=new DataInputStream(conn.getInputStream());
                System.out.println("Successfully connected to "+nxtName);

                try {
                
                    // Thread f�r die Antworten vom NXT
                    Thread mssResponseThread=new Thread(new Runnable() { 
                        public void run() {
                            try {
                                while (true) {
                                    boolean sync=dis.readBoolean();
                                    int cntResult=dis.readInt();
                                    ArrayList<String> result=new ArrayList<String>(cntResult);
                                    for (int i=0;i<cntResult;i++) {
                                        result.add(readString(dis));
                                    }
    
                                    if (sync) {   // Synchrone Message vom Motion-Subsystem
                                        if (initialTestCommand) {
                                            initialTestCommand=false;
                                            if (result.size()==1 && result.get(0).equals("OK")) {
                                                System.out.println("NXT is there, great!");
                                                initialTestSuccessful=true;
                                            }
                                            else {
                                                System.out.println("Communication problems with NXT, unexpected initial response was '"+result.get(0)+"'");
                                            }
                                        }
                                        else {
                                            if (motionSubsystemListener!=null) {

                                                // responseType berechnen
                                                int type=MotionSubsystemListener.RESPONSE_TYPE_OTHERS;
                                                if (result.size()==1) {
                                                    String result0=result.get(0);
                                                    if (result0.equals("OK"))
                                                        type=MotionSubsystemListener.RESPONSE_TYPE_OK;
                                                    else if (result0.startsWith("FAILURE:"))
                                                        type=MotionSubsystemListener.RESPONSE_TYPE_FAILURE;
                                                }
                                            
                                                try {
                                                    motionSubsystemListener.mssResponse(result,type);
                                                }
                                                catch (Exception e) {
                                                    System.out.println("Exception calling mssResponse to "+motionSubsystemListener.getClass().getName()+": "+e);
                                                    e.printStackTrace();
                                                }
                                            }

                                            StringBuffer syncResult=new StringBuffer();
                                            for (int i=0;i<result.size();i++) {
                                                if (i>0)
                                                    syncResult.append("\n");
                                                syncResult.append(result.get(i));                                    
                                            }

                                            resultField.setText(syncResult.toString());
                                            if (controllerInstance==null || !controllerInstance.isRunning()) {
                                                commandField.setText("");
                                            }
                                        }
                                    }

                                    else if (result.size()>0) {   // Asynchrone Message vom Motion-Subsystem
                                    
                                        AsyncMotionMessageBundle resultBundle=null;
                                        try {
                                            resultBundle=new AsyncMotionMessageBundle(result);
                                        }
                                        catch (Exception e) {
                                            System.out.println("Malformed async message: "+e.toString());
                                            e.printStackTrace();
                                            continue;
                                        }

                                        if (motionSubsystemListener!=null) {
                                            try {
                                                motionSubsystemListener.mssAsyncMessages(result,resultBundle);  // Der Listener ist in der Regel der Robot-Controller
                                            }
                                            catch (Exception e) {
                                                System.out.println("Exception calling mssAsyncMessages to "+motionSubsystemListener.getClass().getName()+": "+e);
                                                e.printStackTrace();
                                            }
                                        }

                                        // Neue Position an PositionListener melden
                                        
                                        if (resultBundle.containsPos()) {
                                            long receiveTime=System.currentTimeMillis();

                                            boolean regularPosMessage=resultBundle.containsState();

                                            if (regularPosMessage && lastAsyncMessageTime>0) {   // Regul�re Async-Message, also nicht wegen neuem Kommando oder Kommando-Ende
                                                lastAsyncPeriodic=receiveTime-lastAsyncMessageTime;
                                            }
                                            lastAsyncMessageTime=receiveTime;

                                            asyncValueX=resultBundle.getDouble(AsyncMotionMessage.X);
                                            asyncValueY=resultBundle.getDouble(AsyncMotionMessage.Y);
                                            asyncValueANGLE=resultBundle.getDouble(AsyncMotionMessage.ANG);

                                            double tSpeed=resultBundle.getDouble(AsyncMotionMessage.TSPEED);
                                            double aSpeed=resultBundle.getDouble(AsyncMotionMessage.ASPEED);
                                            double t2Speed=resultBundle.getDouble(AsyncMotionMessage.T2SPEED);
                                            double a2Speed=resultBundle.getDouble(AsyncMotionMessage.A2SPEED);

                                            double asyncValueANGLErad=asyncValueANGLE*Robot.PIDIV180;
                                            sinAsyncValueANGLE=Math.sin(asyncValueANGLErad);
                                            cosAsyncValueANGLE=Math.cos(asyncValueANGLErad);

                                            positionTrigger.triggerNewPosition(receiveTime,  // Der MSSEstimator braucht alle Positionen
                                                                               asyncValueX,asyncValueY,asyncValueANGLE,
                                                                               tSpeed,aSpeed,t2Speed,a2Speed
                                                                              );

                                            complexPositionTrigger.triggerNewPosition(receiveTime,   // Der OpticalFlow muss die Position mit dem Bild synchronisieren oder er nimmt estimated positions
                                                                                      lastAsyncPeriodic,
                                                                                      !regularPosMessage,
                                                                                      asyncValueX,asyncValueY,asyncValueANGLE
                                                                                     );

                                        }

                                        // Message f�r die Anzeige zusammenbauen
                                        StringBuffer asyncResult=new StringBuffer();
                                        for (int i=0;i<result.size();i++) {
                                            String asyncLine=result.get(i);
                                            if (i>0)
                                                asyncResult.append("\n");
                                            asyncResult.append(asyncLine);                                    
                                        }
                                        asyncField.setText(asyncResult.toString());
                                    }
                                }
                            }
                            catch (Exception e) {
                                System.out.println("Stream from NXT terminated");
                            }
                        }
                      });
                      mssResponseThread.start();

            // Sende erstes Kommando (nur ip) zum Test
  
                    try {
                        initialTestCommand=true;
                        writeString(dos,"ip "+ipAddress);
                        dos.flush();
                    }
                    catch (Exception e) {
                        System.out.println("Communication with NXT not successful...");
                        System.out.println(e);
                        e.printStackTrace();
                        System.out.println("Disconnecting and stop");
                        try {conn.close();} catch (Exception e2) {}
                        System.exit(1);
                    }

                    System.out.println("First USB message successful");

// Nicht sch�n aber pragmatisch: Warten bis die erste Nachricht best�tigt wurde
                    for (int i=0;i<50;i++) {
                        try { Thread.sleep(100); } catch (InterruptedException e) {}
                        if (initialTestSuccessful)
                            break;

                    }
                    if (!initialTestSuccessful) {
                        System.out.println("Communication problems with NXT, no response to initial command, very probably NXT is switched off");
                        System.exit(1);
                    }

// Life-Sign-Thread im Simulator nicht notwendig
//                new Thread(new Runnable() { 
//                    public void run() {
//                        while (true) {
//                            try { Thread.sleep(LIFE_SIGN_CYCLE); } catch (Exception e) {}
//                            try {
//                                sendCommand("ping");
//                            }
//                            catch (Exception e) {
//                                System.out.println("Cannot send life sign");
//                            }
//                        }
//                    }
//                  }).start();

                    // Thread, der die aktulle Pose, LCD-Anzeige und Collisionsstatus als Text darstellt
                    new Thread(new Runnable() { 
                        public void run() {
                            try {
                                while (true) {
                                    try {
                                        Thread.sleep(200);
                                    }
                                    catch (InterruptedException e) {}

                                    StringBuffer lcdResult=new StringBuffer();
                                    for (int i=0;i<LCD.text.length;i++) {
                                        if (i>0)
                                            lcdResult.append("\n");
                                        lcdResult.append(LCD.text[i]);                                    
                                    }
                                    lcdField.setText(lcdResult.toString());

                                    StringBuffer poseResult=new StringBuffer();
                                    if (Tetrix.isUp) {
                                        poseResult.append("X="+double2String(DCMotorController.currentPosX,"0.0")+" cm\n");
                                        poseResult.append("Y="+double2String(DCMotorController.currentPosY,"0.0")+" cm\n");
                                        poseResult.append("Angle="+double2String(DCMotorController.currentAngle,"0.0")+" deg\n");
                                        poseResult.append("Speed="+double2String(DCMotorController.currentCMperSec,"0.0")+" cm/s\n");
                                        poseResult.append("Angle speed="+double2String(DCMotorController.currentAngleDegPerSec,"0.0")+" deg/s");
                                        posField.setText(poseResult.toString());

                                        if (simCanvas.collisionsCount>0) {
                                            collisionField.setText(simCanvas.collisionsCount+" collisions over "+simCanvas.totalCollisionsMS+" ms");
                                        }
                                    }
                                    else {
                                        posField.setText("Offline");
                                    }
                                }
                            }
                            catch (Exception e) {
                                System.out.println("Exception in LCD/Pose Thread");
                            }
                        }
                      }).start();
                }
                catch (Exception e) {
                    System.out.println(e);
                    e.printStackTrace();
                    System.exit(1);
                }
            }
        }
        catch (Exception e) {
            System.out.println("Exception in CarbotSim initialization "+e);
            e.printStackTrace();
        }
    }


/**
* Required to implement the simulated motion subsystem. Send a driving command to the motion subsystem.
* @param commandLine the command string, e.g. 'fore 100'
* @throws Exception if something happens
*/
    public void sendCommand(String commandLine) throws Exception {
        synchronized (dos) {
            writeString(dos,commandLine);
            dos.flush();
        }

        if (controllerInstance!=null && controllerInstance.isRunning()) {
//            commandField.setText(StringUtil.limitLength(commandLine,40));
            commandField.setText(commandLine);
        }

    }


/**
* Required to implement the simulated motion subsystem. Register a listener to  asynchronous messages from the motion subsystem.
* @param listener object that is informed when asynchronous messages appear from the motion subsystem.
*/
    public void registerMotionListener(MotionSubsystemListener listener) {
        if (motionSubsystemListener!=null)
            throw new RuntimeException("This Motion Subsystem can only register one listener");
        motionSubsystemListener=listener;
    }


/**
* Required to implement the simulated motion subsystem. Estimate a position (future or past).
* @param timeMillis the time for which the position should be estimated
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if the timestamp is before the earliest position message or there was no position message at all until now.
*/
    public double[] estimatePosition(long timeMillis) {
        return mssEstimator.estimatePosition(timeMillis);
    }


/**
* Required to implement the simulated motion subsystem. Estimate a position of the past.
* @param timeMillis the time for which the position should be estimated
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if one of these cases occur<ul>
*           <li>no position message at all until now</li>
*           <li>the timestamp is before the earliest position message</li>
*           <li>the timestamp is after the newest position message</li>
*         </ul>
*/
    public double[] estimatePastPosition(long timeMillis) {
        return mssEstimator.estimatePastPosition(timeMillis);
    }


/**
* Required to implement the simulated motion subsystem. Estimate a position of the future.
* @param timeMillis the time for which the position should be estimated
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if one of these cases occur<ul>
*           <li>no position message at all until now</li>
*           <li>the timestamp is before the newest position message</li>
*         </ul>
*/
    public double[] estimateFuturePosition(long timeMillis) {
        return mssEstimator.estimateFuturePosition(timeMillis);
    }


/**
* Required to implement the simulated motion subsystem. Estimate a position. The caller configure, if for future, past, or both. Moreover the caller can ask for the respective times stored in the message buffer.
* @param timeMillis the time for which the position should be estimated
* @param past can the timestamp be before the last explicit position message
* @param future can the timestamp be after the last explicit position message
* @param storedTimes the caller can pass a long[2] which is filled with timestamps: [0] earliest position message [1] newest position message;
*                    set to {-1, -1} if there was not position message at all until now.
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if one of these cases occur<ul>
*           <li>no position message at all until now</li>
*           <li>the timestamp is before the earliest position message</li>
*           <li>the timestamp is before the newest position message and past==false</li>
*           <li>the timestamp is after the newest position message and future==false</li>
*         </ul>
*/
    public double[] estimatePosition(long timeMillis,boolean past,boolean future,long[] storedTimes) {
        return mssEstimator.estimatePosition(timeMillis,past,future,storedTimes);
    }


    private static void writeString(DataOutputStream out,String s) throws Exception {
        out.writeInt(s.length());
        for (int i=0;i<s.length();i++)
            out.writeChar(s.charAt(i));
    }

    private static String readString(DataInputStream in) throws Exception {
        int len=in.readInt();
        String result="";
        for (int i=0;i<len;i++) {
            result+=in.readChar();
        }
        return result;
    }



    public String getIPAddress() {
        return "192.168.1.253";
    }

    private void initUI() {

        try {
//            setIconImage(Toolkit.getDefaultToolkit().getImage(CarbotSim.class.getClassLoader().getResource("resources/carbotlogo64x64.png")));
            ArrayList<Image> iconImages=new ArrayList<>();
            iconImages.add(Toolkit.getDefaultToolkit().getImage(CarbotSim.class.getClassLoader().getResource("resources/carbotlogo32x32.png")));
            iconImages.add(Toolkit.getDefaultToolkit().getImage(CarbotSim.class.getClassLoader().getResource("resources/carbotlogo64x64.png")));
            setIconImages(iconImages);
        }
        catch (Exception e) {}


        GridBagLayout gridbag=new GridBagLayout();
        GridBagConstraints c=new GridBagConstraints();

        setLayout(gridbag);  

        c.fill=GridBagConstraints.HORIZONTAL;

// *********** Zoom und Audio-Control **********

        zoomInButton=new JButton("Zoom in");
        c.gridx=0;
        c.gridy=0;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(zoomInButton,c);
        add(zoomInButton);
        zoomInButton.setToolTipText("Get nearer to the bottom");
        zoomInButton.addActionListener(this);

        zoomOutButton=new JButton("Zoom out");
        c.gridx=0;
        c.gridy=1;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(zoomOutButton,c);
        add(zoomOutButton);
        zoomOutButton.setToolTipText("Get farer from the bottom");
        zoomOutButton.addActionListener(this);

        soundCheck=new JCheckBox("Sound");
        c.gridx=1;
        c.gridy=0;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(soundCheck,c);
        add(soundCheck);
        soundCheck.setToolTipText("Enable or disable motor and collision sound");
        soundCheck.addChangeListener(this);

        volCombo=new JComboBox<String>(new String[]{"10 loud","9","8","7","6","5","4","3","2","1 quiet"});
        c.gridx=2;
        c.gridy=0;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(volCombo,c);
        add(volCombo);
        volCombo.setToolTipText("Motor and collision sound volume 10-loud ... 1-quite");
        volCombo.addActionListener(this);

        consoleCheck=new JCheckBox("Debug Out");
        c.gridx=1;
        c.gridy=1;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(consoleCheck,c);
        add(consoleCheck);
        consoleCheck.setToolTipText("Enable or disable debug console");
        consoleCheck.addChangeListener(this);


        followCheck=new JCheckBox("Follow");
        c.gridx=2;
        c.gridy=1;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(followCheck,c);
        add(followCheck);
        followCheck.setToolTipText("Should the screen follow the robot?");
        followCheck.addChangeListener(this);


// *********** Command und Status **********

        JLabel labelCommand=new JLabel("Command (submit with 'Enter')");
        c.gridx=0;
        c.gridy=2;
        c.weightx=0.1;
        c.weighty=0.0;
        c.gridwidth=3;
        c.gridheight=1;
        gridbag.setConstraints(labelCommand,c);
        add(labelCommand);

        commandField=new JTextField();
        commandField.setColumns(20);
        commandField.setHorizontalAlignment(JTextField.LEFT);
        c.gridx=0;
        c.gridy=3;
        c.weightx=0.1;
        c.weighty=0.1;
        c.gridwidth=3;
        c.gridheight=1;
        gridbag.setConstraints(commandField,c);
        add(commandField);
        commandField.addActionListener(this);
        commandField.setToolTipText("Command directly sent to the Motion Subsystem");


        JLabel labelResult=new JLabel("Result");
        c.gridx=0;
        c.gridy=4;
        c.weightx=0.1;
        c.weighty=0.0;
        c.gridwidth=3;
        c.gridheight=1;
        gridbag.setConstraints(labelResult,c);
        add(labelResult);

        resultField=new JTextArea(10,20);
        resultField.setEditable(false);
        resultField.setToolTipText("Command response from the Motion Subsystem");

        JScrollPane resultScrollPane=new JScrollPane(resultField);
        c.gridx=0;
        c.gridy=5;
        c.weightx=0.1;
        c.weighty=1.0;
        c.gridwidth=3;
        c.gridheight=1;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(resultScrollPane,c);
        add(resultScrollPane);

        JLabel labelAsync=new JLabel("Async");
        c.gridx=0;
        c.gridy=6;
        c.weightx=0.1;
        c.weighty=0.0;
        c.gridwidth=3;
        c.gridheight=1;
        gridbag.setConstraints(labelAsync,c);
        add(labelAsync);


        asyncField=new JTextArea(10,20);
        asyncField.setEditable(false);
        asyncField.setToolTipText("Asynchronous messages from the Motion Subsystem");

        JScrollPane asyncScrollPane=new JScrollPane(asyncField);
        c.gridx=0;
        c.gridy=7;
        c.weightx=0.1;
        c.weighty=1.0;
        c.gridwidth=3;
        c.gridheight=1;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(asyncScrollPane,c);
        add(asyncScrollPane);


        JLabel labelLCD=new JLabel("LCD");
        c.gridx=0;
        c.gridy=8;
        c.weightx=0.1;
        c.weighty=0.0;
        c.gridwidth=3;
        c.gridheight=1;
        gridbag.setConstraints(labelLCD,c);
        add(labelLCD);


        lcdField=new JTextArea(7,20);
        lcdField.setEditable(false);
        c.gridx=0;
        c.gridy=9;
        c.weightx=0.1;
        c.weighty=0.1;
        c.gridwidth=3;
        c.gridheight=1;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(lcdField,c);
        add(lcdField);
        lcdField.setToolTipText("Robot's Motion Subsystem LCD field");
        if (remoteDebugging)
            lcdField.setText("-- Wait for LCD during remote debugging --");


        JLabel labelPos=new JLabel("Pose");
        c.gridx=0;
        c.gridy=10;
        c.weightx=0.1;
        c.weighty=0.0;
        c.gridwidth=3;
        c.gridheight=1;
        gridbag.setConstraints(labelPos,c);
        add(labelPos);


        posField=new JTextArea(5,20);
        posField.setEditable(false);
        c.gridx=0;
        c.gridy=11;
        c.weightx=0.1;
        c.weighty=0.1;
        c.gridwidth=3;
        c.gridheight=1;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(posField,c);
        add(posField);
        posField.setToolTipText("Simulated position");


        collisionField=new JTextField("-- no collisions --");
        collisionField.setEditable(false);
        c.gridx=0;
        c.gridy=12;
        c.weightx=0.1;
        c.weighty=0.1;
        c.gridwidth=3;
        c.gridheight=1;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(collisionField,c);
        add(collisionField);
        collisionField.setToolTipText("Robot collision state");
        if (remoteDebugging)
            collisionField.setText("-- No collision state when remote debugging --");


// *********** Canvas und 3D Ansicht **********
        
        canvasPane = new JLayeredPane();

        c.gridx=3;
        c.gridy=0;
        c.weightx=1.0;
        c.weighty=1.0;
        c.gridwidth=20;
        c.gridheight=12;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(canvasPane,c);
        add(canvasPane);
        
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
            	drawCanvasPane();
            }
        });
        
        addComponentListener(new ComponentListener() {
            public void componentResized(ComponentEvent e) {
            	resizeCanvasPane();
            }
			@Override
			public void componentMoved(ComponentEvent e) {}
			@Override
			public void componentShown(ComponentEvent e) {}
			@Override
			public void componentHidden(ComponentEvent e) {}
        });

// *********** Show-Leiste unten **********


        try {
            resetButton=new JButton(new ImageIcon(CarbotSim.class.getClassLoader().getResource("resources/reset.png")));
        }
        catch (Exception e) {
            resetButton=new JButton("Reset");
        }
        c.gridx=3;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=2;
        gridbag.setConstraints(resetButton,c);
        add(resetButton);
        resetButton.addActionListener(this);
        resetButton.setEnabled(controllerInstance!=null);
        resetButton.setToolTipText("Reset all to its initial state");

        try {
            startButton=new JButton(new ImageIcon(CarbotSim.class.getClassLoader().getResource("resources/start.png")));
        }
        catch (Exception e) {
            startButton=new JButton("Start");
        }
        c.gridx=4;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=2;
        gridbag.setConstraints(startButton,c);
        add(startButton);
        startButton.addActionListener(this);
        startButton.setEnabled(controllerInstance!=null || remoteDebugging);
        startButton.setToolTipText("Start Robot Controller");


        try {
            pauseButton=new JButton(new ImageIcon(CarbotSim.class.getClassLoader().getResource("resources/pause.png")));
        }
        catch (Exception e) {
            pauseButton=new JButton("Pause");
        }
        c.gridx=5;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=2;
        gridbag.setConstraints(pauseButton,c);
        add(pauseButton);
        pauseButton.addActionListener(this);
        pauseButton.setEnabled(false);
        pauseButton.setToolTipText("Pause Robot Controller without clearing the state");


        try {
            stopButton=new JButton(new ImageIcon(CarbotSim.class.getClassLoader().getResource("resources/stop.png")));
        }
        catch (Exception e) {
            stopButton=new JButton("Stop");
        }
        c.gridx=6;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=2;
        gridbag.setConstraints(stopButton,c);
        add(stopButton);
        stopButton.addActionListener(this);
        stopButton.setEnabled(false);
        stopButton.setToolTipText("Stop Robot Controller and clear the state");



        debugPaintSelButton=new JButton("DebugOvl");  // <html>Debug<br/>Overlays
        c.gridx=7;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(debugPaintSelButton,c);
        add(debugPaintSelButton);
        debugPaintSelButton.addActionListener(this);
        debugPaintSelButton.setEnabled(controllerInstance!=null || remoteDebugging);
        debugPaintSelButton.setToolTipText("Select the Robot Controller debug painting overlays");



        camLookCombo=new JComboBox<String>(new String[]{"Cam Solid","Cam Glass","Cam Wires","Cam off"});
        c.gridx=7;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(camLookCombo,c);
        add(camLookCombo);
        camLookCombo.addActionListener(this);
        camLookCombo.setToolTipText("Configure the camera view's look");
        camLookCombo.setEnabled(!remoteDebugging);

        camLocCombo=new JComboBox<String>(new String[]{"\u25E4 Up left","\u25E5 Up ri.","\u25E3 Low left","\u25E2 Low ri."," \u25A3 Center"});
        c.gridx=8;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(camLocCombo,c);
        add(camLocCombo);
        camLocCombo.addActionListener(this);
        camLocCombo.setToolTipText("Configure the camera view's location");


        camSizeCombo=new JComboBox<String>(new String[]{"Org","1.5x","2x","2.5x","3x","max"});
        c.gridx=8;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(camSizeCombo,c);
        add(camSizeCombo);
        camSizeCombo.addActionListener(this);
        camSizeCombo.setToolTipText("Configure the camera view's size");

        JLabel labelShow=new JLabel("",SwingConstants.RIGHT);  // Vorher stand da "Show:"
        c.gridx=10;
        c.gridy=12;
        c.weightx=0.1;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=2;
        gridbag.setConstraints(labelShow,c);
        add(labelShow);

        showGridCheck=new JCheckBox("Grid");
        showGridCheck.setSelected(true);
        c.gridx=11;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showGridCheck,c);
        add(showGridCheck);
        showGridCheck.addChangeListener(this);

        showInternalPoseCheck=new JCheckBox("Int.Pose");
        showInternalPoseCheck.setSelected(true);
        c.gridx=11;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showInternalPoseCheck,c);
        add(showInternalPoseCheck);
        showInternalPoseCheck.addChangeListener(this);

        showKeypointsCheck=new JCheckBox("Keypoints");
        showKeypointsCheck.setSelected(true);
        c.gridx=12;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showKeypointsCheck,c);
        add(showKeypointsCheck);
        showKeypointsCheck.addChangeListener(this);

        showCamRangeCheck=new JCheckBox("Cam Range");
        showCamRangeCheck.setSelected(true);
        c.gridx=12;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showCamRangeCheck,c);
        add(showCamRangeCheck);
        showCamRangeCheck.addChangeListener(this);

        showShaddowCheck=new JCheckBox("Shaddow");
        showShaddowCheck.setSelected(true);
        c.gridx=13;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showShaddowCheck,c);
        add(showShaddowCheck);
        showShaddowCheck.addChangeListener(this);

        showSlickCheck=new JCheckBox("Slicks");
        showSlickCheck.setSelected(true);
        c.gridx=13;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showSlickCheck,c);
        add(showSlickCheck);
        showSlickCheck.addChangeListener(this);

        showCollisionsCheck=new JCheckBox("Collisions");
        showCollisionsCheck.setSelected(true);
        c.gridx=14;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showCollisionsCheck,c);
        add(showCollisionsCheck);
        showCollisionsCheck.addChangeListener(this);

        showUSCheck=new JCheckBox("US");
        showUSCheck.setSelected(true);
        c.gridx=14;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showUSCheck,c);
        add(showUSCheck);
        showUSCheck.addChangeListener(this);

        showTrackCheck=new JCheckBox("Track");
        showTrackCheck.setSelected(true);
        c.gridx=15;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showTrackCheck,c);
        add(showTrackCheck);
        showTrackCheck.addChangeListener(this);

        showTyreTracksCheck=new JCheckBox("Tyres");
        showTyreTracksCheck.setSelected(true);
        c.gridx=15;
        c.gridy=13;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(showTyreTracksCheck,c);
        add(showTyreTracksCheck);
        showTyreTracksCheck.addChangeListener(this);


        clearTrackButton=new JButton("<html>Clear<br/>Tracks");
        c.gridx=16;
        c.gridy=12;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=2;
        gridbag.setConstraints(clearTrackButton,c);
        add(clearTrackButton);
        clearTrackButton.addActionListener(this);
        clearTrackButton.setToolTipText("Clear tyre and robot tracks");

        loadScreenConfiguration(); 

        setVisible(true);
        addWindowListener(this);
        doLayout();
        
    }

    private void drawCanvasPane() {
        camera3D = new Camera3D();
        camera3D.setBorder(BorderFactory.createLineBorder(Color.black,6));
        canvasPane.add(camera3D, 1);

    	Bounds parentBounds = new Bounds(0, 0, canvasPane.getWidth(), canvasPane.getHeight());
    	
        simCanvas = new SimCanvas();
        simCanvas.setBounds(parentBounds);
        canvasPane.add(simCanvas, 2);
        
        camera3D.resize(parentBounds, CameraSize.ORIGINAL, CameraLocation.UPPER_LEFT);
        
        Platform.runLater(new Runnable() {
            @Override
            public void run() {
            	List<Shape3D> shapes = Environment3D.getShapes();	
               	camera3D.createScene(shapes);
            }
        });
    }
    
    private void resizeCanvasPane() {
    	javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
            	if(simCanvas != null && camera3D != null) {
            		Bounds parentBounds = new Bounds(0, 0, canvasPane.getWidth(), canvasPane.getHeight());
            		simCanvas.setBounds(parentBounds);
                	camera3D.resize(parentBounds);
            	}
            }
        });
    }

    private void resizeCanvasPane(CameraSize cameraSize) {
    	javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                camera3D.resize(cameraSize);
            }
        });
    }
    
    private void resizeCanvasPane(CameraLocation cameraLocation) {
    	javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                //canvas3D.resize(cameraLocation);
            	camera3D.resize(CameraLocation.UPPER_LEFT); //FOR DEVELOPMENT
            }
        });
    }

    public void actionPerformed(ActionEvent e) {

        if (e.getSource()==zoomOutButton)
            simCanvas.zoomOut();
        else if (e.getSource()==zoomInButton)
            simCanvas.zoomIn();
        else if (e.getSource()==commandField) {
            String commandLine=commandField.getText();
            if (!commandLine.equals("")) {
                simCanvas.disableCopyright();
                try {
                    if (remoteDebugging) {  // Beim Remote-Debugging: das Kommando �ber das Netzwerk schicken
                        outputStreamCmd.writeShort(4); // 4=Commando eingeben
                        outputStreamCmd.writeUTF(commandLine);
                    }
                    else {
                        sendCommand(commandLine);  // Ansonsten an das lokale Motion Subsystem schicken
                    }
                }
                catch (Exception exc) {
                    System.out.println("Cannot send command: "+exc);
                }
            }
        }
        else if (e.getSource()==volCombo) {
            Sound.setVolume(volCombo.getSelectedIndex());
        }
        else if (e.getSource()==camLookCombo) {
            simCanvas.camLook=camLookCombo.getSelectedIndex();
        }
        else if (e.getSource()==camLocCombo) {
        	int index = camLocCombo.getSelectedIndex();
            simCanvas.camLoc = index;
            resizeCanvasPane(CameraLocation.values()[index]);
        }
        else if (e.getSource()==camSizeCombo) {
            int index = camSizeCombo.getSelectedIndex();
            simCanvas.camSize = index;
            resizeCanvasPane(CameraSize.values()[index]);
        }
        else if (e.getSource()==clearTrackButton) {
            simCanvas.clearTrack();
            camera3D.clearTrack();
        }
        else if (e.getSource()==resetButton) {
            simCanvas.disableCopyright();
            environment.setInitialPosition();
            simCanvas.clearTrack();
            camera3D.clearTrack();
            
            if (controllerInstance.isRunning()) {
                controllerInstance.setStateStop();                
                Robot.debugOut.println("**** Send 'stop' to controller "+controllerInstance.getClass().getName());
            }
            else {
                try {                         
                    controllerInstance.stop();
                    Robot.debugOut.println("**** Successfully executed 'stop' by controller "+controllerInstance.getClass().getName());                              
                }
                catch (Exception e2) {
                    Robot.debugOut.println("**** Controller "+controllerInstance.getClass().getName()+" executed 'stop' with exception");
                    Robot.debugOut.printStackTrace(e2);
                }                     
            }
        }
        else if (e.getSource()==startButton) {
            simCanvas.disableCopyright();
  
            commandField.setEnabled(false);
            startButton.setEnabled(false);
            pauseButton.setEnabled(true);
            stopButton.setEnabled(true);

            if (remoteDebugging) {     // Wenn Remote Debugging: �ber das Netzwerk abwickeln
                try {
                    outputStreamCmd.writeShort(1); // 1=Run
                }
                catch (IOException exc) {
                    System.out.println("Cannot send run command to remote host ("+exc.toString()+")");
                }
            }
            else {                     // Ansonsten, beim lokalen Robot-Controller einen Thread anlegen und "run" ausf�hren
                Thread controllerThread=new Thread(new Runnable() {
                      public void run() {
                          try {
                              Robot.debugOut.println("=======================================================");
                              Robot.debugOut.println("**** Start controller "+controllerInstance.getClass().getName());
                              long startTime=System.currentTimeMillis();
                              controllerInstance.setStateRun();
                              controllerInstance.run();
                              long endTime=System.currentTimeMillis();
                              Robot.debugOut.println("**** Controller "+controllerInstance.getClass().getName()+" terminated normally");
                              if (endTime-startTime<200)
                                  Robot.debugOut.println("**** CAUTION: The execution terminated IMMEDIATELY - it is not very likely, the run() method performed any useful controlling");                          
                          }
                          catch (Exception e) {
                              Robot.debugOut.println("**** Controller "+controllerInstance.getClass().getName()+" terminated with exception");
                              Robot.debugOut.printStackTrace(e);
                          }
                          
                          try {Thread.sleep(100);} catch (InterruptedException e) {}
                              
                          if (controllerInstance.isPaused()) {
                              try {
                                  controllerInstance.pause();
                                  Robot.debugOut.println("**** Successfully executed 'pause' by controller "+controllerInstance.getClass().getName());                              
                              }
                              catch (Exception e) {
                                  Robot.debugOut.println("**** Controller "+controllerInstance.getClass().getName()+" executed 'pause' with exception");
                                  Robot.debugOut.printStackTrace(e);
                              }                                  
                          }
                          else {
                              try {                         
                                  controllerInstance.setStateStop();
                                  controllerInstance.stop();
                                  Robot.debugOut.println("**** Successfully executed 'stop' by controller "+controllerInstance.getClass().getName());                              
                              }
                              catch (Exception e) {
                                  Robot.debugOut.println("**** Controller "+controllerInstance.getClass().getName()+" executed 'stop' with exception");
                                  Robot.debugOut.printStackTrace(e);
                              }                                   
                          }

                          commandField.setEnabled(true);
                          try { Thread.sleep(1); } catch (InterruptedException e) {}   // Braucht man leider, sonst kommt es vor, dass setText ein "Beep" verursacht
                          commandField.setText("");

                          startButton.setEnabled(true);
                          pauseButton.setEnabled(false);
                          stopButton.setEnabled(false);
                      }
                     });
                controllerThread.start();
                
                camera3D.startSimulation();
            }
        }
        else if (e.getSource()==pauseButton) {
            pauseButton.setEnabled(false);
            stopButton.setEnabled(false);

            camera3D.stopSimulation();

            if (remoteDebugging) {     // Wenn Remote Debugging: �ber das Netzwerk abwickeln
                try {
                    outputStreamCmd.writeShort(2); // 2=Pause
                }
                catch (IOException exc) {
                    System.out.println("Cannot send pause command to remote host ("+exc.toString()+")");
                }

            }
            else {                     // Ansonsten, beim lokalen Robot-Controller "pause" ausf�hren
                try {
                    controllerInstance.setStatePause();
                    Robot.debugOut.println("**** Successfully executed 'pause' by controller "+controllerInstance.getClass().getName());                    
                }
                catch (Exception exc) {
                    Robot.debugOut.println("**** Controller "+controllerInstance.getClass().getName()+" executed 'pause' with exception");
                    Robot.debugOut.printStackTrace(exc);                   
                }
            }
        }
        else if (e.getSource()==stopButton) {
            pauseButton.setEnabled(false);
            stopButton.setEnabled(false);

            camera3D.stopSimulation();

            if (remoteDebugging) {     // Wenn Remote Debugging: �ber das Netzwerk abwickeln
                try {
                    outputStreamCmd.writeShort(3); // 3=Stop
                }
                catch (IOException exc) {
                    System.out.println("Cannot send stop command to remote host ("+exc.toString()+")");
                }

            }
            else {                     // Ansonsten, beim lokalen Robot-Controller "stop" ausf�hren
                controllerInstance.setStateStop();
                Robot.debugOut.println("**** Send 'stop' to controller "+controllerInstance.getClass().getName());
            }
        }
        else if (e.getSource()==debugPaintSelButton) {
            DebugPainterOverlaySelectionDialog.openDialog(this);
        }
    }


    private void setRemoteshowKeypoints(boolean enabled) {
        try {
            outputStreamCmd.writeShort(5); // 5=enable/disable show Keypoints
            outputStreamCmd.writeBoolean(enabled); 
        }
        catch (IOException exc) {
            System.out.println("Cannot send enable/disable keypoints command to remote host ("+exc.toString()+")");
        }
    }


    public void stateChanged(ChangeEvent e) {
        if (e.getSource()==soundCheck) {
            volCombo.setEnabled(soundCheck.isSelected());
            Sound.setSoundOnOff(soundCheck.isSelected());
            if (soundCheck.isSelected());
                Sound.setVolume(volCombo.getSelectedIndex());

        }
        else if (e.getSource()==consoleCheck) {
            if (consoleCheck.isSelected())
                simDebugConsole.showConsole();
            else
                simDebugConsole.hideConsole();
        }
        else if (e.getSource()==followCheck) {
             simCanvas.setFollowRobot(followCheck.isSelected());
        }
        else if (e.getSource()==showGridCheck) {
            simCanvas.showGrid=showGridCheck.isSelected();
        }
        else if (e.getSource()==showInternalPoseCheck) {
            simCanvas.showInternalPose=showInternalPoseCheck.isSelected();
        }
        else if (e.getSource()==showKeypointsCheck) {
            simCanvas.showKeypoints=showKeypointsCheck.isSelected();
            if (remoteDebugging) {
                setRemoteshowKeypoints(simCanvas.showKeypoints);
            }
        }
        else if (e.getSource()==showCamRangeCheck) {
            simCanvas.showCamRange=showCamRangeCheck.isSelected();
        }
        else if (e.getSource()==showShaddowCheck) {
            simCanvas.showShaddow=showShaddowCheck.isSelected();
        }
        else if (e.getSource()==showSlickCheck) {
            simCanvas.showSlick=showSlickCheck.isSelected();
        }
        else if (e.getSource()==showCollisionsCheck) {
            simCanvas.showCollisions=showCollisionsCheck.isSelected();
        }
        else if (e.getSource()==showUSCheck) {
            simCanvas.showUS=showUSCheck.isSelected();
        }
        else if (e.getSource()==showTrackCheck) {
            simCanvas.showTrack=showTrackCheck.isSelected();
        }
        else if (e.getSource()==showTyreTracksCheck) {
            simCanvas.showTyreTracks=showTyreTracksCheck.isSelected();
        }
    }


    public void windowDeactivated(WindowEvent e) {
    }


    public void windowActivated(WindowEvent e) {
    }


    public void windowIconified(WindowEvent e) {
       if (controllerInstance!=null && controllerInstance.isRunning()) {
           setState(JFrame.NORMAL);
           JOptionPane.showMessageDialog(null, "Iconizing disabled during simulation run!\nIconized applications do not get sufficient CPU power on some systems","Error",JOptionPane.ERROR_MESSAGE);
       }
    }


    public void windowDeiconified(WindowEvent e) {
    }


    public void windowClosing(WindowEvent e) {
        saveScreenConfiguration();
        System.exit(0);
    }


    public void windowOpened(WindowEvent e) {
        commandField.requestFocusInWindow();
    }


    public void windowClosed(WindowEvent e) {
    }



    private void saveScreenConfiguration() {
        try {
            ObjectOutputStream out=new ObjectOutputStream(new FileOutputStream(new File(SCREENCONFIGFILE)));
            Dimension d=getSize();

            // Screen-Dimensions
            out.writeInt(d.width);
            out.writeInt(d.height);

            // Checkboxes
            out.writeBoolean(soundCheck.isSelected());
            out.writeInt(volCombo.getSelectedIndex());
            out.writeInt(camLookCombo.getSelectedIndex());
            out.writeInt(camLocCombo.getSelectedIndex());
            out.writeInt(camSizeCombo.getSelectedIndex());
            out.writeBoolean(showGridCheck.isSelected());
            out.writeBoolean(showInternalPoseCheck.isSelected());
            out.writeBoolean(showKeypointsCheck.isSelected());
            out.writeBoolean(showCamRangeCheck.isSelected());
            out.writeBoolean(showShaddowCheck.isSelected());
            out.writeBoolean(showSlickCheck.isSelected());
            out.writeBoolean(showCollisionsCheck.isSelected());
            out.writeBoolean(showUSCheck.isSelected());
            out.writeBoolean(showTrackCheck.isSelected());
            out.writeBoolean(showTyreTracksCheck.isSelected());

            // Debug Console
            out.writeBoolean(consoleCheck.isSelected());
            int[] consoleScreenConfig=simDebugConsole.getScreenConfiguration();        
            out.writeInt(consoleScreenConfig[0]);
            out.writeInt(consoleScreenConfig[1]);
            out.writeInt(consoleScreenConfig[2]);
            out.writeInt(consoleScreenConfig[3]);

            // DebugPainterOverlays
            HashSet<String> disabled=((SimDebugPainter)Robot.debugPainter).getOverlayDisabled();
            out.writeInt(disabled.size());
            for (String dis:disabled) {
                out.writeUTF(dis);
            }
            
            out.flush();
            out.close();
        }
        catch (Exception e) {
            System.out.println("Cannot save screen configurtion ("+e.toString()+")");
        }
    }


    private void loadScreenConfiguration() {

        ObjectInputStream in=null;

        try {
            in=new ObjectInputStream(new FileInputStream(new File(SCREENCONFIGFILE)));

            // Screen-Dimensions
            int screen_width=in.readInt();
            int screen_height=in.readInt();
            setSize(screen_width,screen_height);


            // Checkboxes
            Sound.shouldPlaySound=in.readBoolean();
            int vol=in.readInt();

            simCanvas.camLook=in.readInt();
            simCanvas.camLoc=in.readInt();
            simCanvas.camSize=in.readInt();
            simCanvas.showGrid=in.readBoolean();
            simCanvas.showInternalPose=in.readBoolean();
            simCanvas.showKeypoints=in.readBoolean();
            simCanvas.showCamRange=in.readBoolean();
            simCanvas.showShaddow=in.readBoolean();
            simCanvas.showSlick=in.readBoolean();
            simCanvas.showCollisions=in.readBoolean();
            simCanvas.showUS=in.readBoolean();
            simCanvas.showTrack=in.readBoolean();
            simCanvas.showTyreTracks=in.readBoolean();

            soundCheck.setSelected(Sound.shouldPlaySound);
            soundCheck.setEnabled(Sound.canPlaySound);
            volCombo.setSelectedIndex(vol);
            volCombo.setEnabled(Sound.shouldPlaySound & Sound.canPlaySound);
            Sound.setVolume(vol);

            camLookCombo.setSelectedIndex(simCanvas.camLook);
            camLocCombo.setSelectedIndex(simCanvas.camLoc);
            camSizeCombo.setSelectedIndex(simCanvas.camSize);

            showGridCheck.setSelected(simCanvas.showGrid);
            showInternalPoseCheck.setSelected(simCanvas.showInternalPose);
            showKeypointsCheck.setSelected(simCanvas.showKeypoints);
            showCamRangeCheck.setSelected(simCanvas.showCamRange);
            showShaddowCheck.setSelected(simCanvas.showShaddow);
            showSlickCheck.setSelected(simCanvas.showSlick);
            showCollisionsCheck.setSelected(simCanvas.showCollisions);
            showUSCheck.setSelected(simCanvas.showUS);
            showTrackCheck.setSelected(simCanvas.showTrack);
            showTyreTracksCheck.setSelected(simCanvas.showTyreTracks);
        }
        catch (Exception e) {
            System.out.println("Error reading Screen Configuration (using default) "+e);
            setSize(1200,950);
        }


        try {
            // Debug Console
            boolean showConsole=in.readBoolean();
            int consolePosX=in.readInt();
            int consolePosY=in.readInt();
            int consoleSizeX=in.readInt();
            int consoleSizeY=in.readInt();

            simDebugConsole.setScreenConfiguration(consolePosX,consolePosY,consoleSizeX,consoleSizeY);
            consoleCheck.setSelected(showConsole);
        }
        catch (Exception e) {
            System.out.println("Error reading Debug Console Screen Configuration (using default) "+e);

            simDebugConsole.setScreenConfiguration(30,30,300,300);
        }


        try {
            // DebugPainterOverlays
            int cntDis=in.readInt();
            HashSet<String> disabled=new HashSet<>();
            for (int i=0;i<cntDis;i++)
                disabled.add(in.readUTF());
            ((SimDebugPainter)Robot.debugPainter).setOverlayDisabled(disabled);

        }
        catch (Exception e) {
            System.out.println("Error reading Debug Overlays Screen Configuration (using default) "+e);
        }


        try {
            in.close();
        }
        catch (Exception e) {
        }

        if (remoteDebugging) {
            setRemoteshowKeypoints(simCanvas.showKeypoints);
        }
    }


    public void setConsoleCheck(boolean selected) {
        consoleCheck.setSelected(selected);
    }


/**
* Formats a double as string using a certain format (e.g. 0.000).
* @param num the double
* @param format the format
* @return formatted string
*/
    public static String double2String(double num,String format) {
        DecimalFormat form=new DecimalFormat(format,new DecimalFormatSymbols(Locale.US));
        StringBuffer result=new StringBuffer();
        form.format(num,result,new FieldPosition(0));
        return result.toString();
    }



    private void waitForRemoteDebuggingHost(String remoteDebuggingHost) throws IOException {
        InetAddress hostIP=InetAddress.getByName(remoteDebuggingHost);

        System.out.println("Connect to "+remoteDebuggingHost+" for debugging connection (remote commands)...");
        Socket socketCmd=new Socket(hostIP,6548);
        inputStreamCmd=new DataInputStream(socketCmd.getInputStream());
        outputStreamCmd=new DataOutputStream(socketCmd.getOutputStream());

        System.out.println("Connect to "+remoteDebuggingHost+" for debugging connection (remote debug painter)...");
        Socket socketDebug=new Socket(hostIP,6549);
        inputStreamDebug=new DataInputStream(socketDebug.getInputStream());

        System.out.println("Connect to "+remoteDebuggingHost+" for debugging connection (motion subsystem messages)...");
        Socket socketMSS=new Socket(hostIP,6550);
        inputStreamMSS=new DataInputStream(socketMSS.getInputStream());

        System.out.println("Connect to "+remoteDebuggingHost+" for debugging connection (camera image)...");
        Socket socketCam=new Socket(hostIP,6551);
        inputStreamCam=new DataInputStream(socketCam.getInputStream());


        System.out.println("Debugging connections to "+remoteDebuggingHost+" established");
    }


    private void createInputThreads() { 

        Thread cmdInThread=new Thread(new Runnable() {
             public void run() {
                 try {
                     while (true) {
                         short cmd=inputStreamCmd.readShort();
                         switch (cmd) {
                             case 11:  // 11=State->Stop AFTER RUN
                                 System.out.println("Remote Controller terminated normally");
                                 commandField.setEnabled(true);
                                 commandField.setText("");

                                 startButton.setEnabled(true);
                                 pauseButton.setEnabled(false);
                                 stopButton.setEnabled(false);

                                 break;

                             case 12:  // 12=State->Pause
                                 System.out.println("Remote Controller paused normally");

                                 break;

                             case 13:  // 13=State->Stop
                                 System.out.println("Remote Controller stopped normally");

                                 break;


                             case 21:  // 21=terminate with error
                                 String errorMsg=inputStreamCmd.readUTF();
                                 System.out.println("Remote Controller terminated with error ("+errorMsg+")");

                                 break;

                             case 22:  // 22=pause with error
                                 errorMsg=inputStreamCmd.readUTF();
                                 System.out.println("Remote Controller paused with error ("+errorMsg+")");

                                 break;

                             case 23:  // 23=stop with error
                                 errorMsg=inputStreamCmd.readUTF();
                                 System.out.println("Remote Controller stopped with error ("+errorMsg+")");
                                 break;

                             default: 
                                 System.out.println("Error reading debug CMD input stream, illegal cmd "+cmd);
                                 
                         }
                     }
                 }
                 catch (IOException e) {
                     System.out.println("Error reading debug CMD input stream ("+e.toString()+")");
                 }
             }
          });
        cmdInThread.start();

        Thread mssInThread=new Thread(new Runnable() {
             public void run() {
                 try {
                     while (true) {
                         short cmd=inputStreamMSS.readShort();
                         short cnt=inputStreamMSS.readShort();
                         ArrayList<String> result=new ArrayList<>(cnt);
                         for (int i=0;i<cnt;i++)
                             result.add(inputStreamMSS.readUTF());


                         switch (cmd) {
                             case 101: // 101=Sync Response
                                 StringBuffer syncResult=new StringBuffer();
                                 for (int i=0;i<result.size();i++) {
                                     if (i>0)
                                         syncResult.append("\n");
                                     syncResult.append(result.get(i));                                    
                                 }
                                 String syncResultStr=syncResult.toString();
                                 
                                 if (syncResultStr.startsWith("LCDOUT:")) {  // LCDout ist ein Sonderfall, hier nicht auf die Result-Console sondern auf den LCD-Screen schreiben
                                     lcdField.setText(syncResultStr.substring(8)); // LDCOUT:<newline>
                                 }
                                 else {
                                     resultField.setText(syncResultStr);
                                     commandField.setText("");
                                 }
                                 break;

                             case 102: // 102=ASync Response
                                 StringBuffer asyncResult=new StringBuffer();

                                 AsyncMotionMessageBundle resultBundle=null;
                                 try {
                                     resultBundle=new AsyncMotionMessageBundle(result);
                                 }
                                 catch (Exception e) {
                                     System.out.println("Malformed async message for message #102: "+e.toString());
                                     e.printStackTrace();
                                 }

                                 if (resultBundle.containsPos() && resultBundle.containsType(AsyncMotionMessage.US)) {
                                     asyncValueX=resultBundle.getDouble(AsyncMotionMessage.X);
                                     asyncValueY=resultBundle.getDouble(AsyncMotionMessage.Y);
                                     asyncValueANGLE=resultBundle.getDouble(AsyncMotionMessage.ANG);

                                     double asyncValueANGLErad=asyncValueANGLE*Robot.PIDIV180;
                                     sinAsyncValueANGLE=Math.sin(asyncValueANGLErad);
                                     cosAsyncValueANGLE=Math.cos(asyncValueANGLErad);

                                     double asyncTSPEED=resultBundle.getDouble(AsyncMotionMessage.TSPEED);
                                     double asyncASPEED=resultBundle.getDouble(AsyncMotionMessage.ASPEED);
                                     double usDist=resultBundle.getDouble(AsyncMotionMessage.US);

                                     StringBuffer poseResult=new StringBuffer();
                                     poseResult.append("X="+double2String(asyncValueX,"0.0")+" cm\n");
                                     poseResult.append("Y="+double2String(asyncValueY,"0.0")+" cm\n");
                                     poseResult.append("Angle="+double2String(asyncValueANGLE,"0.0")+" deg\n");
                                     poseResult.append("Speed="+double2String(asyncTSPEED,"0.0")+" cm/s\n");
                                     poseResult.append("Angle speed="+double2String(asyncASPEED,"0.0")+" deg/s");

                                     posField.setText(poseResult.toString());

                                     Tetrix.currentPosX=asyncValueX;
                                     Tetrix.currentPosY=asyncValueY;
                                     Tetrix.currentAngle=asyncValueANGLE;

                                     DCMotorController.currentPosX=asyncValueX;
                                     DCMotorController.currentPosY=asyncValueY;
                                     DCMotorController.currentAngle=asyncValueANGLE;

                                     if (usDist<200) {
                                         double usPointX=asyncValueX+sinAsyncValueANGLE*Robot.usDist;
                                         double usPointY=asyncValueY+cosAsyncValueANGLE*Robot.usDist;
                                         double usHitPointX=usPointX+sinAsyncValueANGLE*usDist;
                                         double usHitPointY=usPointY+cosAsyncValueANGLE*usDist;
                                         Environment.lastUSHit=new double[][]{
                                            {usPointX,usPointY},
                                            {usHitPointX,usHitPointY}
                                         };
                                     }
                                     else {
                                         Environment.lastUSHit=null;
                                     }
                                 }

                                 for (int i=0;i<result.size();i++) {
                                     String asyncLine=result.get(i);
                                     if (i>0)
                                         asyncResult.append("\n");
                                     asyncResult.append(asyncLine);                                    
                                 }

                                 asyncField.setText(asyncResult.toString());
                                 break;
                                 
                             default: 
                                 System.out.println("Error reading debug MSS input stream, illegal cmd "+cmd);
                                 
                         }
                     }
                 }
                 catch (IOException e) {
                     System.out.println("Error reading debug MSS input stream ("+e.toString()+")");
                 }
             }
          });
        mssInThread.start();
        

        Thread camInThread=new Thread(new Runnable() {
             public void run() {
                 try {
                     while (true) {
                         int len=inputStreamCam.readInt();
                         byte[] buf=new byte[len];
                         int off=0;
                         while (off<buf.length) {
                             int cnt=inputStreamCam.read(buf,off,buf.length-off);
                             if (cnt<0)
                                 throw new IOException("EOF reading camera image after "+off+"/"+buf.length+" bytes");
                             off+=cnt;
                         }

                         ByteArrayInputStream in=new ByteArrayInputStream(buf);
                         BufferedImage image=ImageIO.read(in);
                                 
                         simCanvas.setRemoteDebugCameraImage(image);
                     }
                 }
                 catch (IOException e) {
                     System.out.println("Error reading debug Cam input stream ("+e.toString()+")");
                 }
             }
          });
        camInThread.start();        
    }


}