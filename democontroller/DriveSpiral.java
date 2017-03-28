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
import robotinterface.vss.VisionSubsystemListener2D;
import robotinterface.vss.VisionSubsystemListener3D;
import robotinterface.vss.KeyPointPackage2D;
import robotinterface.vss.KeyPointPackage3D;
import robotinterface.vss.ObservedKeyPoint2D;
import robotinterface.vss.ObservedKeyPoint3D;

import robotinterface.util.Matrix;


/**
* A demo controller that drives a spiral until a collision is detected.
* While driving, 2D keypoints are collected and transformed into 3D.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class DriveSpiral extends RobotController implements MotionSubsystemListener,VisionSubsystemListener2D,VisionSubsystemListener3D {

    private int radius=30;  // Spiralen-Radius - wird beim Fahren ständig erhöht

    private boolean anyCollision=false;
    private boolean tactile=false;
    private double us=Double.NaN;
    private double posX=Double.NaN;
    private double posY=Double.NaN;
    private double posAng=Double.NaN;



// Für Demo: 2D-Keypoints selbst in 3D umwandeln
    private final static double MIN_SQR_POSDIST_FOR_VIEW2D=100; // Minimale Positionsdifferenz zweier 2D-Keypoints, um daraus einen 3D-Punkt berechnen zu können
    private HashMap<Integer, ArrayList<ObservedKeyPoint2D>> sequences=new HashMap<Integer, ArrayList<ObservedKeyPoint2D>>();  // Alle durchgehend empfangenen 2D-Keypoints, LineID->Liste der 2D Keypoints



    public DriveSpiral() {
        Robot.motionSubsystem.registerMotionListener(this);

        Robot.visionSubsystem.setTiming(VisionSubsystem.TRIGGERED,-1);


        try {
            Robot.visionSubsystem.registerVisionListener2D(this);
        }
        catch (UnsupportedOperationException e) {
            Robot.debugOut.println("Vision Subsystem does not provide 2D keypoints");
        }


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
               "Drive a spiral until a collision is detected";
    }


/**
* Drive a spiral until isRunning()==false.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void run() throws Exception {
        Robot.motionSubsystem.sendCommand("stoprule T,U50");

        while (isRunning() && !anyCollision) {
            Robot.debugOut.println("Drive circle with radius "+radius);
            Robot.motionSubsystem.sendCommand("right "+radius+" 720");

            radius+=5;
            Thread.sleep(3000);          
        }
    }


/**
* Pause driving a spiral. Keep the last circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void pause() throws Exception {
        Robot.motionSubsystem.sendCommand("stop");
    }


/**
* Stop driving a spiral. Reset the circle radius.
* @throws Exception if something happens, usually a problem of the robot controller instance
*/
    public void stop() throws Exception {
        Robot.motionSubsystem.sendCommand("stop");
        radius=30;
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
    
        boolean newPos=false;
        if (bundle.containsPos()) { 
            posX=bundle.getDouble(AsyncMotionMessage.X);
            posY=bundle.getDouble(AsyncMotionMessage.Y);
            posAng=bundle.getDouble(AsyncMotionMessage.ANG);
            newPos=true;
        }   
        
        if (bundle.containsType(AsyncMotionMessage.TACTIL)) 
            tactile=bundle.getBoolean(AsyncMotionMessage.TACTIL);

        if (bundle.containsType(AsyncMotionMessage.US)) 
            us=bundle.getDouble(AsyncMotionMessage.US);      
        
        if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL) || 
            bundle.containsType(AsyncMotionMessage.COLLISION_US)) {
             anyCollision=true;
        }           

        // Malen der Position auf über den Debug-Painter
        if (newPos) {
            Robot.debugPainter.setOverlay("Carbot position");

            Robot.debugPainter.clear();

            Robot.debugPainter.fillSquare(posX,posY,5,0,0,150,255);
            Robot.debugPainter.drawLine(posX,posY,posX-100,posY+100,0,0,150,255);

            Robot.debugPainter.drawText(posX-100,posY+110,"MSS ("+Math.round(posX)+", "+Math.round(posY)+", "+Math.round(posAng)+"\u00b0)",true,true,9,0,0,150,255);
            Robot.debugPainter.paint();
        }
    }


/**
* Required to implement the <i>VisionSubsystemListener2D</i>. A list of keypoints (2D) is passed for further computation.
* This demo code shows, how to get 3D positions for observed 2D pixelpositions with the help
* of a overdetermined system of linear equations.
* @param keypoints2D keypoints (2D)
* @throws Exception if something happens
*/
    public void observedKeypoints2D(KeyPointPackage2D keypoints2D) throws Exception {

 // Menge alle Keypoint-IDs machen
        HashSet<Integer> occuredKeypoints=new HashSet<Integer>(); 
        for (ObservedKeyPoint2D obskp: keypoints2D.observedKeyPoints) {
            occuredKeypoints.add(obskp.lineID);
        }


 // Alle Sequenzen löschen, die nicht mehr in der neuen Messung vorkommen
        ArrayList<Integer> toRemove=new ArrayList<Integer>(sequences.size());  // Leider kann man nicht gleichzeitig durchlaufen und löschen, daher Liste aufbauen, was später gelöscht werden soll
        for (Integer lineID:sequences.keySet()) {
            if (!occuredKeypoints.contains(lineID))
                toRemove.add(lineID);
        }
        for (Integer rmv:toRemove) { // Alle vorher gemerkten löschen
            sequences.remove(rmv);
        }

 // Jetzt alle Messungen mit den gespeicherten Sequenzen verrechnen
 // - kommt diese ID vor, Sequenz erweitern
 // - kommt diese ID nicht vor, eine neue Sequenz eintragen

        ArrayList<double[]> computedKeyPoints3D=new ArrayList<double[]>();   // Liste der berechnete 3D-Punkte

        for (int i=0;i<keypoints2D.observedKeyPoints.size();i++) {
            ObservedKeyPoint2D obskp=keypoints2D.observedKeyPoints.get(i);

            ArrayList<ObservedKeyPoint2D> seq=sequences.get(obskp.lineID);
            if (seq==null) {           // Diese Sequenz gab es noch nicht
                seq=new ArrayList<ObservedKeyPoint2D>();
                seq.add(obskp);
                sequences.put(obskp.lineID,seq);
            }
            else {                     // Diese Sequenz gab es schon
                                       // Suche von allen bisherigen Punkten denjenige aus, der am weitesten vom letzen Punkt entfernt ist
                                       // Das ist der Observation-Punkt 1, der andere ist der aktuelle
                double maxSqrDist=0.0;
                ObservedKeyPoint2D maxKpo=null;
                for (ObservedKeyPoint2D kpo:seq) {
                    double sqrDist=(kpo.observationPosX-obskp.observationPosX)*(kpo.observationPosX-obskp.observationPosX)+
                                   (kpo.observationPosY-obskp.observationPosY)*(kpo.observationPosY-obskp.observationPosY);
                    if (sqrDist>maxSqrDist) {
                        maxSqrDist=sqrDist;
                        maxKpo=kpo;
                    }
                }
                seq.add(obskp);  // Hinzufügen der letzten Position

                if (maxSqrDist>MIN_SQR_POSDIST_FOR_VIEW2D) {          // Die aktuelle Position ist weit genug von der entferntesten Aufnahmeposition weg
                    double[] computedKeyPoint3D=keypoints2Dto3D(maxKpo,obskp,true);  // Hinter dieser Funktion steht die eigentlich Arbeit mit dem Gleichungssystem
                    if (computedKeyPoint3D!=null) {
                        computedKeyPoints3D.add(computedKeyPoint3D);
                    }
                }
            }
        }


// Jetzt noch die erkannten Punkte malen über den Debug-Painter malen

        Robot.debugPainter.setOverlay("Keypoints from 2D, computed to 3D");
        Robot.debugPainter.clear();
        for (int i=0;i<computedKeyPoints3D.size();i++) {
            double[] computedKeyPoint3D=computedKeyPoints3D.get(i);
            Robot.debugPainter.fillCircle(computedKeyPoint3D[0],computedKeyPoint3D[1],computedKeyPoint3D[3],255,0,0,100);
            Robot.debugPainter.drawCross(computedKeyPoint3D[0],computedKeyPoint3D[1],20,0,0,0,255);
        }
        Robot.debugPainter.paint();

    }


// Aus zwei beobachteten Pixel-Positionen eine 3D-Position berechnen
// Wenn computeDeviation==true wird zusätzlich ein [3] mit der möglichen Abweichung in cm berechnet
    private static double[] keypoints2Dto3D(ObservedKeyPoint2D kp1,ObservedKeyPoint2D kp2,boolean computeDeviation) {


// sin, cos der Winkel braucht man häufiger

        double sina1=Math.sin(kp1.observationAngle*Robot.PIDIV180);
        double cosa1=Math.cos(kp1.observationAngle*Robot.PIDIV180);
        double sina2=Math.sin(kp2.observationAngle*Robot.PIDIV180);
        double cosa2=Math.cos(kp2.observationAngle*Robot.PIDIV180);

// Die 2 Rotationsmatrizen rotMatrixTotal1, rotMatrixTotal2 berechnen 

        double[][] rotMatrixRobotOrientation1=Matrix.rotationMatrix3DZ(sina1,cosa1);
        double[][] rotMatrixRobotOrientation2=Matrix.rotationMatrix3DZ(sina2,cosa2);
        double[][] rotMatrixCamera=Matrix.rotationMatrix3DX(Robot.sinCamAlpha,Robot.cosCamAlpha);
        double[][] rotMatrixTotal1=Matrix.multiply(rotMatrixCamera,rotMatrixRobotOrientation1);   // Rotationsmatrizen werden von rechts nach links angewendet, d.h.
        double[][] rotMatrixTotal2=Matrix.multiply(rotMatrixCamera,rotMatrixRobotOrientation2);   // erst die Robot-Orientierung rückgängig machen, dann die Kamera-Neigung zurückdrehen


// Die 2 Kamera-Positionen cam1, cam2 berechnen
        double camX1=kp1.observationPosX+sina1*Robot.camDist;
        double camY1=kp1.observationPosY+cosa1*Robot.camDist;
        double camZ1=Robot.camHeight;
        double[] cam1=new double[]{camX1,camY1,camZ1};

        double camX2=kp2.observationPosX+sina2*Robot.camDist;
        double camY2=kp2.observationPosY+cosa2*Robot.camDist;
        double camZ2=Robot.camHeight;
        double[] cam2=new double[]{camX2,camY2,camZ2};

// Damit man nicht um den 0-Punkt drehen muss (könnte bei entfernten Roboterfahrten zu entarteten Matrizen führen)
// Tue ich so, als ob der 0-Punkt der Mittelpunkt der beiden Kamera-Positionen ist
// Diesen Mittelpunkt muss ich dem Ergebnis (X, Y, Z) dazuzählen 
        double[] camCenter=new double[]{(camX1+camX2)/2,(camY1+camY2)/2,(camZ1+camZ2)/2};
 
        cam1=Matrix.sub(cam1,camCenter);
        cam2=Matrix.sub(cam2,camCenter);

// Jetzt kann man das Gleichungssystem aufstellen, zuerst Matrix A

        double[][] A=new double[6][5];
        Matrix.copyInto(A,0,0,rotMatrixTotal1);
        Matrix.copyInto(A,3,0,rotMatrixTotal2);
        A[0][3]=-kp1.undistortedNormalizedX;
        A[1][3]=-1.0d;
        A[2][3]=-kp1.undistortedNormalizedY;

        A[3][4]=-kp2.undistortedNormalizedX;
        A[4][4]=-1.0d;
        A[5][4]=-kp2.undistortedNormalizedY;

// dann Konstantenvektor B

        double[] T1=Matrix.multiply(rotMatrixTotal1,cam1);   // Achtung: Kamera-Positionen erst drehen
        double[] T2=Matrix.multiply(rotMatrixTotal2,cam2);

        double[] B=new double[6];
        Matrix.copyInto(B,0,T1);
        Matrix.copyInto(B,3,T2);

// Jetzt das überbestimmte Gleichungssystem lösen

        try {
            double[] solution=Matrix.solveOverdeterminedLinearEquation(A,B);

            if (computeDeviation) {  // Es soll eine Abweichung geschätzt werden
                A[0][3]=-kp1.undistortedNormalizedX+1/Robot.cam_fx;   // Das mache ich so, indem ich beide Pixel-Werte um einen Pixel verschiebe, der erste (+1,+1)
                A[2][3]=-kp1.undistortedNormalizedY+1/Robot.cam_fy;

                A[3][4]=-kp2.undistortedNormalizedX-1/Robot.cam_fx;   // der zweite (-1,-1)
                A[5][4]=-kp2.undistortedNormalizedY-1/Robot.cam_fy;

                double[] solution2=Matrix.solveOverdeterminedLinearEquation(A,B);                // und das mit den verschobenen Pixel nochmal löse; Achtung: doppelte Laufzeit!

                double deviation=Math.sqrt((solution[0]-solution2[0])*(solution[0]-solution2[0])+(solution[1]-solution2[1])*(solution[1]-solution2[1])+(solution[2]-solution2[2])*(solution[2]-solution2[2]));
                
                return new double[]{solution[0]+camCenter[0],solution[1]+camCenter[1],solution[2]+camCenter[2],deviation};           
            }
            return new double[]{solution[0]+camCenter[0],solution[1]+camCenter[1],solution[2]+camCenter[2]};
        }
        catch (IllegalArgumentException e) {   // Gleichungssystem nicht lösbar
            return null;
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
//        if (isRunning())
//            Robot.motionSubsystem.sendCommand("offset "+(keypoints3D.observationPosX-keypoints3D.mssPosX)+" "+(keypoints3D.observationPosY-keypoints3D.mssPosY)+" "+(keypoints3D.observationAngle-keypoints3D.mssAngle));


        Robot.debugPainter.setOverlay("Keypoints from 3D (only info!)");
        Robot.debugPainter.clear();

        Robot.debugPainter.fillSquare(keypoints3D.observationPosX,keypoints3D.observationPosY,5,0,100,0,255);
        Robot.debugPainter.drawLine(keypoints3D.observationPosX,keypoints3D.observationPosY,keypoints3D.observationPosX+100,keypoints3D.observationPosY+100,0,100,0,255);
        Robot.debugPainter.drawText(keypoints3D.observationPosX+100,keypoints3D.observationPosY+110,"(SLAM "+Math.round(keypoints3D.observationPosX)+", "+Math.round(keypoints3D.observationPosY)+", "+Math.round(keypoints3D.observationAngle)+"\u00b0)",true,true,9,0,100,0,255);


        for (int i=0;i<keypoints3D.observedKeyPoints.size();i++) {
            ObservedKeyPoint3D obskp=keypoints3D.observedKeyPoints.get(i);
            Robot.debugPainter.drawCross(obskp.x,obskp.y,20,0,100,0,255);
        }
        Robot.debugPainter.paint();
    }


}