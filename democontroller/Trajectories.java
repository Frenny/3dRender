package democontroller;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;


import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.mss.MotionSubsystem;
import robotinterface.mss.MotionSubsystemListener;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;

import robotinterface.vss.VisionSubsystemListener2D;
import robotinterface.vss.VisionSubsystemListener3D;
import robotinterface.vss.KeyPointPackage2D;
import robotinterface.vss.KeyPointPackage3D;
import robotinterface.vss.ObservedKeyPoint2D;
import robotinterface.vss.ObservedKeyPoint3D;
import robotinterface.util.Matrix;
import robotinterface.util.GeomUtil;

import robotlib.traj.Trajectory;
import robotlib.traj.MiniTrajectorySequence;
import robotlib.traj.TrajectorySequence;


/**
* A demo controller that drives trajectories.
* Each run drives another trajectory, i.e. multiples re-runs iterate through all possible trajectories.
* This controller requires a configuration &quot;&lt;trargetX&gt;;&lt;trargetY&gt;).
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Trajectories extends RobotController implements MotionSubsystemListener {


    private double targetX=Double.NaN;
    private double targetY=Double.NaN;

    private boolean anyCollision=false;
    private boolean tactile=false;
    private double us=Double.NaN;
    private double posX=Double.NaN;
    private double posY=Double.NaN;
    private double posAng=Double.NaN;

    private TrajectorySequence driveSeq=null;
    private int seqNr=-1;


    public Trajectories() {
        Robot.motionSubsystem.registerMotionListener(this);
    }


/**
* Returns a short description of this robot controller, e.g. about the author and configuration.
* @return (multiline) string description
*/
    public String getDescription() {
        return "Robot controller "+getClass().getName()+":\n"+
               "Drive trajectories from point to point";
    }


/**
* Indicates, whether the robot controller requires a configuration. If so, the developer has to override the configure() method.
* @return true if this controllers requires a configuration
*/
    public boolean requiresConfiguration() {
        return true;
    }


/**
* Pass a configuration to the robot controller. This method is executed directly after instantiation and before calling the run() the first time.
* The controller has to memorize the configuration for further calls of run().
* The configuration is a controller-dependent string, usually passed as command-line argument. The entire parsing has to be performed by the controller.
* @param params a configuration string in controller-dependent format
* @throws IllegalArgumentException if either the controller does not accept a configuration or the string is malformed
*/
    public void configure(String params) throws IllegalArgumentException {
        String[] split=params.split(";");
        if (split.length!=2)
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" requires configuration <targetx>;<targety>, but '"+params+"' does not provide two fields");

        try {
            targetX=new Double(split[0]).doubleValue();
        }
        catch (NumberFormatException e) {
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" requires configuration <targetx>;<targety>, but '"+split[0]+"' is not number");
        }

        try {
            targetY=new Double(split[1]).doubleValue();
        }
        catch (NumberFormatException e) {
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" requires configuration <targetx>;<targety>, but '"+split[1]+"' is not number");
        }
    }


/**
* Navigate until isRunning()==false.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void run() throws Exception {

        Robot.motionSubsystem.sendCommand("stoprule T,U50");
        Robot.debugOut.println("Compute trajectories...");

        KeyPointPackage3D keyPointPackage3D=Robot.visionSubsystem.getAllKeypoints3D();

        double startAngle=GeomUtil.mssAngle2NavAngle(keyPointPackage3D.observationAngle);


        ArrayList<MiniTrajectorySequence> trajectoryVariations=MiniTrajectorySequence.computeAllVariations(

                          keyPointPackage3D.observationPosX,
                          keyPointPackage3D.observationPosY,
                          startAngle,
                          keyPointPackage3D.observationPosX+targetX,
                          keyPointPackage3D.observationPosY+targetY,
                          startAngle,

                          MiniTrajectorySequence.FLAGS_ALLOW_BACKDRIVING | 
                          MiniTrajectorySequence.FLAGS_ARCS_UPTO_360_DEGREES | 
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE,     
                          30.0,1000.0,1.0,
                          10.0,new double[]{1.5d, 3.0d});

        Robot.debugOut.println("Found "+trajectoryVariations.size()+" variations to get to target/angle");

        seqNr=(seqNr+1) % trajectoryVariations.size();       

        Robot.debugOut.println("Take Variation #"+seqNr);

        driveSeq=trajectoryVariations.get(seqNr);
        Robot.debugOut.println(driveSeq.dumpStr());

        Robot.debugOut.println("Trajectory sequence with "+driveSeq.size()+" trajectories found");

        Trajectory first=driveSeq.pollFirst();
        first.paint(255,100,100,255);
        Robot.debugPainter.paint();


        Robot.debugOut.println("First MSS command: "+first.getMSSCommand());
        Robot.motionSubsystem.sendCommand(first.getMSSCommand());

        while (isRunning() && !anyCollision && driveSeq!=null) {
            Thread.sleep(500);          
        }
    }


/**
* Pause driving a spiral. Keep the last circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void pause() throws Exception {
        Robot.motionSubsystem.sendCommand("stop");
        driveSeq=null;
    }


/**
* Stop driving a spiral. Reset the circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void stop() throws Exception {
        Robot.motionSubsystem.sendCommand("stop");
        driveSeq=null;
        anyCollision=false;
    }


/**
* Required to implement the <i>MotionSubsystemListener</i>. A (synchronous) response message appears from the motion subsystem. Response messages are a result of a 
* command sent to the motion subsystem.
* @param messages list of strings indicating the response, usually only a single, e.g. 'OK'
* @param responseType one of RESPONSE_TYPE_OK, RESPONSE_TYPE_FAILURE, RESPONSE_TYPE_OTHERS
* @throws Exception if something happens
*/
    public void mssResponse(ArrayList<String> messages,int responseType) throws Exception {
        if (responseType==MotionSubsystemListener.RESPONSE_TYPE_FAILURE)
             Robot.debugOut.println("Failure response "+messages.get(0));
    }


/**
* Required to implement the <i>MotionSubsystemListener</i>. An asynchronous message appears from the motion subsystem. Asynchronous message are periodically issued
* and indicate the current state and position.
* @param messages list of strings indicating the current state
* @param bundle the parsed messages that easily provides access to key and values
* @throws Exception if something happens
*/
    public void mssAsyncMessages(ArrayList<String> messages,AsyncMotionMessageBundle bundle) throws Exception {
    
        if (bundle.containsPos()) {
            posX=bundle.getDouble(AsyncMotionMessage.X);
            posY=bundle.getDouble(AsyncMotionMessage.Y);
            posAng=bundle.getDouble(AsyncMotionMessage.ANG);
        }    
        
        if (bundle.containsType(AsyncMotionMessage.TACTIL)) 
            tactile=bundle.getBoolean(AsyncMotionMessage.TACTIL);

        if (bundle.containsType(AsyncMotionMessage.US)) 
            us=bundle.getDouble(AsyncMotionMessage.US);       
    
        if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL) || 
            bundle.containsType(AsyncMotionMessage.COLLISION_US)) {
             anyCollision=true;
             driveSeq=null;
        }
       
        if (bundle.containsType(AsyncMotionMessage.STOPPED)) {
            if (driveSeq==null) {
                Robot.debugOut.println("MSS command queue is null");
            }
            else {
                Trajectory next=driveSeq.pollFirst();
                if (next==null) {
                    Robot.debugOut.println("No more MSS commands");
                    driveSeq=null;
                }
                else {
                    Robot.debugOut.println("Next MSS command: "+next.getMSSCommand());
                    next.paint(255,100,100,255);
                    Robot.debugPainter.paint();
                    Robot.motionSubsystem.sendCommand(next.getMSSCommand());
                }
            }        
        }       
    }


}