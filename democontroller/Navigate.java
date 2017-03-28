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

import robotinterface.vss.VisionSubsystem;
import robotinterface.vss.VisionSubsystemListener3D;
import robotinterface.vss.KeyPointPackage3D;
import robotinterface.vss.ObservedKeyPoint3D;
import robotinterface.util.Matrix;
import robotinterface.util.GeomUtil;

import robotlib.traj.Trajectory;
import robotlib.traj.TurnInPlaceTrajectory;
import robotlib.traj.LinearTrajectory;
import robotlib.traj.MiniTrajectorySequence;
import robotlib.traj.TrajectorySequence;
import robotlib.nav.Grid_Astar;
import robotlib.nav.Pos2PosRouting;
import robotlib.driver.TrajectorySequenceDriver;


/**
* A demo controller that navigates.
* This controller requires a configuration &quot;&lt;trargetX&gt;;&lt;trargetY&gt;).
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Navigate extends RobotController implements MotionSubsystemListener,VisionSubsystemListener3D {

    private final static double MIN_OBSTACLE_HEIGHT=1.0d;   // Min...
    private final static double MAX_OBSTACLE_HEIGHT=100.0d; // ... max Höhe von Keypoints, damit sie als Hindernis gewertet werden


    private double targetX=Double.NaN;     // Ziel-Position X, gegeben als Config-Param
    private double targetY=Double.NaN;     // Ziel-Position Y, gegeben als Config-Param
    private double targetAngleDeg=Double.NaN; // Ziel-Winkel, gegeben als Config-Param (oder NaN) in Grad, wie in der Environment-Datei)
    private double targetAngle=Double.NaN; // Ziel-Winkel, gegeben als Config-Param (oder NaN) (in Rad, wie im Pos2Pos-System)
    private int mode=-1;                   // Fahrmodus, gegeben als Config-Param, 1: once, 2: inc

    private boolean usCollision=false;      // Letzte asynchrone MSS-Meldung war Kollision (US)
    private boolean tactilCollision=false;  // Letzte asynchrone MSS-Meldung war Kollision (Tactile)

    private boolean tactile=false;          // Letzter Tatcil-Wert über asynchrone MSS-Meldung
    private double us=Double.NaN;           // Letzte US-Entfernung über asynchrone MSS-Meldung

    private double posXMSS=Double.NaN;        // Letzte Position X über asynchrone MSS-Meldung
    private double posYMSS=Double.NaN;        // Letzte Position Y über asynchrone MSS-Meldung
    private double posAngMSS=Double.NaN;      // Letzter Winkel über asynchrone MSS-Meldung


    private boolean freshPosition=false;        // Ist die Position nach einer Kollision wieder asynchron durch das MSS erneuert worden?

//    private double posXVSS=Double.NaN;        // Letzte Position X über asynchrone VSS-Meldung (korrigiert)
//    private double posYVSS=Double.NaN;        // Letzte Position Y über asynchrone VSS-Meldung (korrigiert)
//    private double posAngVSS=Double.NaN;      // Letzter Winkel über asynchrone VSS-Meldung (korrigiert)

    private boolean trajectoryNotFree=false;  // Bei der letzten asynchronen VSS-Meldung wurde entdeckt, dass die aktuelle Trajektorie nicht mehr frei ist


    private Pos2PosRouting p2p=null;            // Die letzte Instanz des Routing-Objekte (damit die letzte geplante Route)

    private TrajectorySequenceDriver dsd=null;
    private boolean driveCollisionTrajectory=false;  // Wird gerade die Ausweich-Trajektorie gefahren?

    private double[][] obstacles=null;          // Alle gefundenen Hinderniss-Punkte entweder alle direkt am Anfang (mode=1: once) oder incrementell erweitert (mode=2: inc);
                                                // Achtung: nur 2D


    public Navigate() {
        Robot.motionSubsystem.registerMotionListener(this);
        dsd=new TrajectorySequenceDriver(
                                         90, // rotateSpeed,
                                         22, // travelSpeedArc,
                                         30, // travelSpeedArcFast,
                                         5,  // travelSpeedArcBack,
                                         50, // minFastArcRadius,
                                         50, // minFastArcLength,
                                         25, // travelSpeedLinear,
                                         30, // travelSpeedLinearFast,
                                         10, // travelSpeedLinearBack,
                                         50  // minFastLinearLength
                                        );


        // Achtung: auf dem realen Carbot ist 3D visioning nicht verfügbar. Im Simulator kann das aber verwendet werden,
        // um die echte 3D-Position von Keypoints in die Karte einzuzeichnen. Niemals drüfen die 3D-Punkte für eigene Rechnungen verwendet werden!
        try {
            Robot.visionSubsystem.registerVisionListener3D(this);
        }
        catch (UnsupportedOperationException e) {
            Robot.debugOut.println("Vision Subsystem does not provide 3D keypoints");
        }
    }


/**
* Returns a short description of this robot controller, e.g. about the author and configuration.
* @return (multiline) string description
*/
    public String getDescription() {
        return "Robot controller "+getClass().getName()+":\n"+
               "Navigate from point to point\n"+
               "Configuration <targetX>;<targetY>;<targetAngle>;<mode>\n"+
               "  targetX: target x in cm\n"+
               "  targety: target y in cm\n"+
               "  targetAngle: target angle in degree or 'NaN' for any angle\n"+
               "  mode: 'once': read all obstacles once, 'inc': incremental routing";
    }


/**
* Indicates, that this controller requires a configuration. 
* @return true if this controllers requires a configuration; here, always true
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
        if (split.length!=4)
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" requires configuration <targetX>;<targetY>;<targetAngle>;<mode>, but '"+params+"' does not provide these fields");

        try {
            targetX=new Double(split[0]).doubleValue();
        }
        catch (NumberFormatException e) {
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" failes to read configuration: "+split[0]+"' is not number");
        }

        try {
            targetY=new Double(split[1]).doubleValue();
        }
        catch (NumberFormatException e) {
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" failed to read configuration: "+split[1]+"' is not number");
        }

        if (!split[2].equalsIgnoreCase("nan")) {
            try {
                targetAngleDeg=new Double(split[2]).doubleValue();
                targetAngle=GeomUtil.mssAngle2NavAngle(targetAngleDeg);
            }
            catch (NumberFormatException e) {
                throw new IllegalArgumentException("Robot controller "+getClass().getName()+" failed to read configuration: "+split[2]+"' neither is a number nor 'NaN'");
            }
        }

        if (split[3].equalsIgnoreCase("once"))
            mode=1;
        else if (split[3].equalsIgnoreCase("inc"))
            mode=2;
        else
            throw new IllegalArgumentException("Robot controller "+getClass().getName()+" failed to read configuration: mode must be 'once' or 'inc', but is '"+split[3]+"'");
    }


/**
* Navigate until isRunning()==false.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void run() throws Exception {
        Robot.motionSubsystem.sendCommand("stoprule T,U50");
        Robot.motionSubsystem.sendCommand("rotaterule T,U30");
        Robot.motionSubsystem.sendCommand("periodic 200");

        Robot.visionSubsystem.setTiming(VisionSubsystem.TRIGGERED,-1);

        double firstPosX=Double.NaN;
        double firstPosY=Double.NaN;
        double firstAngle=Double.NaN;

        if (mode==1) {   // Once Mode
            KeyPointPackage3D keyPointPackage3D=Robot.visionSubsystem.getAllKeypoints3D();
            obstacles=keyPointPackage3D.getPositionAsArray(MIN_OBSTACLE_HEIGHT,MAX_OBSTACLE_HEIGHT);   // von 1..100 cm werden keypoints ausgelesen
            paintObstacles();

            firstPosX=keyPointPackage3D.observationPosX;
            firstPosY=keyPointPackage3D.observationPosY;
            firstAngle=keyPointPackage3D.observationAngle;
        }
        else {          // Inc Mode
            while (Double.isNaN(posAngMSS)) {   // Busy-Waiting-Loop um zu warten, bis die asnychr. MSS-Meldung die erste Position belegt hat
                Thread.sleep(100);
            }
            firstPosX=posXMSS;
            firstPosY=posYMSS;
            firstAngle=posAngMSS;
            if (obstacles==null)     // Damit es nach "pause" nicht nochmal angelegt wird
                obstacles=new double[0][];
        }

        double startAngle=GeomUtil.mssAngle2NavAngle(firstAngle);

        Robot.debugOut.println("Compute route...");

        TrajectorySequence firstRouteTraj=computeRouteAndTrajectories(firstPosX,firstPosY,startAngle);
        if (firstRouteTraj==null) return;

        paintTrajectories(firstRouteTraj);
        dsd.drive(firstRouteTraj);
        paintCurrentTrajectory();


        while (isRunning() && !((usCollision || tactilCollision) && mode==1) && !targetReached()) {
            Thread.sleep(100);

            if (mode==2) {   // Nur im Inc-Mode malen, da im Once-Mode alle Hindernisse direkt am Anfang gemalt werden
                paintObstacles();

                if (usCollision || tactilCollision) { 
                    if (!freshPosition) continue;

                    if (usCollision) {
                        obstacles=fusionObstacles(obstacles,posXMSS, posYMSS, posAngMSS, us); 
                    }

                    Robot.debugOut.println("Compute Trajectories because of US collision");
                    TrajectorySequence collTraj=computeCollisionTrajectory();

                    paintTrajectories(collTraj);
                    dsd.drive(collTraj);
                    driveCollisionTrajectory=true;
                    paintCurrentTrajectory();

                    usCollision=false;
                    tactilCollision=false;
                }

                if (dsd.isHalted()) {  // Es gibt keine Trajectorie , z.B. es wurde erkannt, dass die aktuelle Trajektorien nicht mehr befahrbar ist -> neu Routen
                    if (!freshPosition) continue;

                    if (targetReached()) continue;

                    Robot.debugOut.println("Compute NEW route...");

                    TrajectorySequence routeTraj=computeRouteAndTrajectories(posXMSS,posYMSS,GeomUtil.mssAngle2NavAngle(posAngMSS));

                    if (routeTraj==null) return;

                    paintTrajectories(routeTraj);
                    dsd.drive(routeTraj);
                    driveCollisionTrajectory=false;
                    paintCurrentTrajectory();
                    trajectoryNotFree=false;
                }
            }
        }

        if (!isRunning()) {
            if (isPaused())
                Robot.debugOut.println("Controller manually paused");
            else if (isStopped())
                Robot.debugOut.println("Controller manually stopped");
        }
        else if (targetReached())
            Robot.debugOut.println("Target position reached!");

    }


    private TrajectorySequence computeCollisionTrajectory() {
        TrajectorySequence  routeTraj=new TrajectorySequence();

        double currentAng=GeomUtil.mssAngle2NavAngle(posAngMSS);


        // 10 cm rückfahren
        double backX=posXMSS-Math.cos(currentAng)*20;
        double backY=posYMSS-Math.sin(currentAng)*20;
        LinearTrajectory linBack=LinearTrajectory.createTrajectory(posXMSS,posYMSS,backX,backY);
        linBack.driveBackward=true;
        routeTraj.add(linBack);


        double newAng=GeomUtil.reduceAngle(currentAng+Math.PI/3);
        TurnInPlaceTrajectory tip=TurnInPlaceTrajectory.createTrajectory(backX,backY,currentAng,newAng);
        routeTraj.add(tip);

        double newX=backX+Math.cos(newAng)*100;
        double newY=backY+Math.sin(newAng)*100;
        LinearTrajectory lin=LinearTrajectory.createTrajectory(backX,backY,newX,newY);
        routeTraj.add(lin);


//        TurnInPlaceTrajectory tip2=TurnInPlaceTrajectory.createTrajectory(newX,newY,newAng,currentAng);
//        routeTraj.add(tip2);

        return routeTraj;
    }


    private TrajectorySequence computeRouteAndTrajectories(double pX,double pY,double pA) throws Exception {
        int astar_flags=Grid_Astar.FLAGS_STEPWIDTH5 | Grid_Astar.FLAGS_DONT_CUT_CORNER | Grid_Astar.FLAGS_REMOVE_COLLINEAR_AND_BYPASSES | Grid_Astar.FLAGS_CONSIDER_ADDITIONAL_COSTS;

        p2p=new Pos2PosRouting(
                               obstacles,
                               30,   // obstacleBuffer
                               50,   // additionalCostsBuffer
                               10,   // cellSize
                               astar_flags,      // A*-Flags
                               2.0d, // maxAdditionalCostsFactor
                               3.0d, // acceptedShortcutCosts
                               pX,
                               pY,
                               targetX,
                               targetY
                              );

        double[][] route=p2p.getRoute();
        if (route==null) {
            Robot.debugOut.println("No route found!");
            return null;
        }

        paintRoute(p2p);

        Robot.debugOut.println("Found route with "+route.length+" route points");

        Robot.debugOut.println("Compute trajectories...");

        int[] flags_trajectory=null; 
        if (mode==1) {        // Mode=Once
            if (Double.isNaN(targetAngle)) {         // Ziel-Winkel beliebig
                 flags_trajectory=new int[] {
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_ALLOW_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_UPTO_360_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITHOUT_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITHOUT_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES
                         };
            }
            else {
                 flags_trajectory=new int[] {         // Ziel-Winkel vorgegeben
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_ALLOW_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_UPTO_360_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_ALLOW_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_UPTO_360_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_ALLOW_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_UPTO_360_DEGREES
                         };
            }
        }
        else if (mode==2) {        // Mode=Inc
            if (Double.isNaN(targetAngle)) {         // Ziel-Winkel beliebig
                 flags_trajectory=new int[] {
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITHOUT_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITHOUT_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES
                         };
            }
            else {
                 flags_trajectory=new int[] {         // Ziel-Winkel vorgegeben
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES,
                          MiniTrajectorySequence.FLAGS_ALL_WITH_TARGET_ANGLE | MiniTrajectorySequence.FLAGS_NO_BACKDRIVING | MiniTrajectorySequence.FLAGS_ARCS_LESS_180_DEGREES
                         };
            }
        }

        double[] intArcsFactors=new double[]{1.0d,3.0d,5.0d};
        double[] intArcsCostMultipliers=new double[]{1.5d,1.0d,1.0d};

        int flags_trajectory_planning=Pos2PosRouting.FLAGS_TRAJ_PLANNING_ANGLE_ALL | Pos2PosRouting.FLAGS_TRAJ_PLANNING_ALGO_VITERBI;

        TrajectorySequence routeTraj=p2p.trajectoryPlanning(route,pA,
                                         targetAngle, // Kann auch NaN sein
                                         Robot.minCurveRadius+1.0d,   // minCurveRadius
                                         Robot.maxCurveRadius,        // maxCurveRadius
                                         Robot.minDrivingDistance,    // minLength
                                         10.0,                        // maxTrajectoryStretch
                                         intArcsFactors,   
                                         intArcsCostMultipliers,
                                         flags_trajectory,
                                         flags_trajectory_planning,
                                         500,2,   // constantTurnInPlaceCosts,relativeTurnInPlaceCosts,
                                         100,3    // constantBackwardCosts,relativeBackwardCosts
                                        );

        if (routeTraj==null) {
            Robot.debugOut.println("No route trajectories found!");
            return null;
        }

        Robot.debugOut.println("Found trajectries with "+routeTraj.size()+" trajectories");
        return routeTraj;
    }


    private boolean targetReached() {
        if (Math.hypot(posXMSS-targetX,posYMSS-targetY)>0.5)
            return false;
        if (Double.isNaN(targetAngleDeg))
            return true;
        return GeomUtil.degreeDelta(targetAngleDeg,posAngMSS)<0.5;
    }


    private void paintObstacles() {
        if (obstacles==null)
            return;
        Robot.debugPainter.setOverlay("Obstacles");
        Robot.debugPainter.clear();
        for (int i=0;i<obstacles.length;i++)
             Robot.debugPainter.fillCircle(obstacles[i][0],obstacles[i][1],10,0,0,0,255);
        Robot.debugPainter.paint();
    }


    private void paintRoute(Pos2PosRouting p2p) {
        Robot.debugPainter.setOverlay("Route");
        Robot.debugPainter.clear();
        p2p.paintRoute(100,100,200,255);
        Robot.debugPainter.paint();
    }


/**
* Pause the navigation. Keep the last circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void pause() throws Exception {
        dsd.halt();
    }


/**
* Stop the navigation. Reset the circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void stop() throws Exception {
        dsd.halt();
        usCollision=false;
        tactilCollision=false;
        freshPosition=false;
        obstacles=null;
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
            posXMSS=bundle.getDouble(AsyncMotionMessage.X);
            posYMSS=bundle.getDouble(AsyncMotionMessage.Y);
            posAngMSS=bundle.getDouble(AsyncMotionMessage.ANG);
            
            freshPosition=true;            
        }
        
        if (bundle.containsType(AsyncMotionMessage.TACTIL)) 
            tactile=bundle.getBoolean(AsyncMotionMessage.TACTIL);

        if (bundle.containsType(AsyncMotionMessage.US)) 
            us=bundle.getDouble(AsyncMotionMessage.US);
        
        if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL)) {
            Robot.debugOut.println("Collision TACTIL");

            tactilCollision=true;
            freshPosition=false;
            dsd.halt();
        }
        
        if (bundle.containsType(AsyncMotionMessage.COLLISION_US)) {
            Robot.debugOut.println("Collision US");

            usCollision=true;
            freshPosition=false;
            dsd.halt();
        }        
        
        if (bundle.containsType(AsyncMotionMessage.STOPPED)) {
            dsd.mssStoppedReceived();
            paintCurrentTrajectory();
            freshPosition=false;        
        }
    }


/**
* Required to implement the <i>VisionSubsystemListener3D</i>.  A list of keypoints (3D) is passed for further computation.
* Caution: for information only - this feature is not available on the real Carbot. None of the information passed via this call should be used for own computations!
* @param keypoints3D keypoints (3D)
* @throws Exception if something happens
*/
    public void observedKeypoints3D(KeyPointPackage3D keypoints3D) throws Exception {

// Korrektur der MSS-Position anhand der SLAM-Position
        if (isRunning())
            Robot.motionSubsystem.sendCommand("offset "+(keypoints3D.observationPosX-keypoints3D.mssPosX)+" "+(keypoints3D.observationPosY-keypoints3D.mssPosY)+" "+(keypoints3D.observationAngle-keypoints3D.mssAngle));


//        posXVSS=keypoints3D.observationPosX;
//        posYVSS=keypoints3D.observationPosY;
//        posAngVSS=keypoints3D.observationAngle;

        if (mode==2) {   // Nur wenn Inc Mode, da im Once Mode alle Keypoints direkt am Anfang eingelesen werden

            obstacles=fusionObstacles(obstacles,keypoints3D,p2p);


            if (!dsd.isHalted() && p2p!=null) {

                double[] collision=p2p.trajectoryCollision(dsd.currentTrajectory());
                if (collision==null) {
                    TrajectorySequence furtherTraj=dsd.furtherTrajectories();
                    if (furtherTraj!=null && !driveCollisionTrajectory)
                        collision=p2p.trajectoryCollision(furtherTraj);
                }
                if (collision!=null) {  // Die aktuelle Trajektorie oder die restliche Sequenz ist nicht frei

                    Robot.debugPainter.setOverlay("Collision");
                    Robot.debugPainter.clear();
                    Robot.debugPainter.drawCross(collision[0],collision[1],30,0,0,0,255);
                    Robot.debugPainter.paint();

                    double distToCollision=Math.hypot(collision[0]-posXMSS,collision[1]-posYMSS);


                    double minDistToCollision= driveCollisionTrajectory ? 40 : 100;

                    if (distToCollision>minDistToCollision)
                        return;

                    dsd.halt();

                    trajectoryNotFree=true;
                    freshPosition=false;
                }
            }
        }
    }



// Hinzufügen von Kollision bezüglich Ultraschall
    private static double[][] fusionObstacles(double[][] oldObstacles,double posX, double posY, double posAngle, double usDist) {
        double[][] usObstacles=GeomUtil.computeUltrasonicPoints(posX,posY,posAngle,usDist,5);

        if (oldObstacles==null)
            oldObstacles=new double[0][];

        // Erstmal alle alten Hindernispunkte nehmen
        ArrayList<double[]> newObstacles=new ArrayList<>();
        for (int i=0;i<oldObstacles.length;i++) {
            double[] obstacle=oldObstacles[i];
            newObstacles.add(obstacle);
        }

        // Dann alle neuen US-Hindernisspunkte dazunehmen
        for (int i=0;i<usObstacles.length;i++) {
            double[] obstacle=usObstacles[i];
            newObstacles.add(obstacle);
        }

        // Am Ende die ArrayList in ein Feld umkopieren
        double[][] newObstaclesArray=new double[newObstacles.size()][];
        for (int i=0;i<newObstaclesArray.length;i++)
            newObstaclesArray[i]=newObstacles.get(i);
        return newObstaclesArray;
    }


    // Hinzufügen der neue Keypoints zu den alten WENN diese neu sind, d.h. sich hinreichend von jedem in der bisherigen Liste unterscheiden
    // Das wird bisher OHNE räumlichen Index gemacht, deshalb sehr ineffizient - hier könnte man viel verbessern
    private static double[][] fusionObstacles(double[][] oldObstacles,KeyPointPackage3D keypoints3D,Pos2PosRouting p2p) {

        if (oldObstacles==null)
            oldObstacles=new double[0][];

        // Erstmal alle alten Hindernispunkte nehmen
        ArrayList<double[]> newObstacles=new ArrayList<>();
        for (int i=0;i<oldObstacles.length;i++) {
            double[] obstacle=oldObstacles[i];
            newObstacles.add(obstacle);
        }

        // Dann alle neuen Hindernisspunkte durchlaufen
loopI: for (int i=0;i<keypoints3D.observedKeyPoints.size();i++) {
            ObservedKeyPoint3D obskp=keypoints3D.observedKeyPoints.get(i);

            if (obskp.z<MIN_OBSTACLE_HEIGHT || obskp.z>MAX_OBSTACLE_HEIGHT)   // Wenn Keypoint nicht im erwarteten Höhenbereich liegt
                continue;                                                     // ignorieren

            // Und mit den alten vergleichen
            for (int j=0;j<oldObstacles.length;j++) {
                double[] obstacle=oldObstacles[j];

                if (Math.hypot(obskp.x-obstacle[0],obskp.y-obstacle[1])<1.0d) {   // <1cm Unterschied heißt: "identisch"
                    continue loopI;  // Diesen Punkt gab es schon
                }
            }
            newObstacles.add(new double[]{obskp.x,obskp.y});  // Den Punkt gab es noch nicht -> hinzunehmen

            if (p2p!=null)                        // Wenn es schon eine Route gab (und damit ein Grid)
                p2p.addObstacle(obskp.x,obskp.y); // Das neue Hindernis auch dort eintragen, damit man auf "freie Trajektorie" planen kann
        }

        // Am Ende die ArrayList in ein Feld umkopieren
        double[][] newObstaclesArray=new double[newObstacles.size()][];
        for (int i=0;i<newObstaclesArray.length;i++)
            newObstaclesArray[i]=newObstacles.get(i);
        return newObstaclesArray;
    }


    private void paintCurrentTrajectory() {
        Trajectory currentTraj=dsd.currentTrajectory();
        if (currentTraj==null)
            return;

        Robot.debugPainter.setOverlay("Current trajectory");
        Robot.debugPainter.clear();

        currentTraj.paint(255,255,100,255);
        Robot.debugPainter.paint();

        Robot.debugOut.println("MSS command: "+currentTraj.getMSSCommand());
        Robot.debugOut.println("Trajectory: "+currentTraj.dumpStr());
    }


    private void paintTrajectories(TrajectorySequence trajSeq) {
        Robot.debugPainter.setOverlay("Trajectories");
        Robot.debugPainter.clear();
        trajSeq.paint(255,100,100,255);
        Robot.debugPainter.paint();
    }

}