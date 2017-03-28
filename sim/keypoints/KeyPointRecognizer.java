package sim.keypoints;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;

import com.vividsolutions.jts.geom.Geometry;


import robotinterface.Robot;
import robotinterface.vss.VisionSubsystem;
import robotinterface.vss.ObservedKeyPoint2D;
import robotinterface.vss.ObservedKeyPoint3D;
import robotinterface.vss.KeyPointPackage2D;
import robotinterface.vss.KeyPointPackage3D;
import robotinterface.vss.VisionSubsystemListener3D;
import robotinterface.vss.VisionSubsystemListener2D;

import sim.customsim.Customize2DKeypoints;
import sim.customsim.Customize3DKeypoints;


import sim.util.ComplexPositionListener;
import sim.util.ComplexPositionTrigger;
import sim.geom.Slice;
import sim.geom.Projection;
import sim.environment.Environment;


import jr.motion.DCMotorController;  // Nur für die echte Position für die KeypointPackage3D


/**
* An object that collects 3D keypoints (formerly computed by the Canvas for painting) and computes a set of 3D keypoints for the robot controller (if it registers to listen to them).
* The robot controller 'thinks' these keypoints came from computations with 2D pixels as input. As a result,
* 3D key points are only passed to a controller, if they appear in two <i>subsequent</i> camera images from <i>different</i> positions.
* <p>It also passes a list of 2D keypoints to a controller, if the controller registers to listen for 2D keypoints.
* In this case, the 2D keypoints are the 2D projections of the 3D points to the camera plane.
* </p>
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class KeyPointRecognizer implements ComplexPositionListener,VisionSubsystem {

    private final static double MIN_SQR_DIST_FOR_VIEW=100;  // Quadrat des Mindestabstands zweier Aufnahmepositionen, damit ein Keypoint erkannt werden kann
    private final static long MS_FOR_NEW_KEYPOINTS=15000;   // Nach welcher Zeit soll der 2D-Mechanismus die Keypoints neu suchen (d.h. die alten lineID sind ungültig)

    public static double KEYPOINT3DVSTDDEV=0.0d;          // Für die 3d-Keypoints: eine Standardabweichung angeben
    private Random random=new Random();                    // Für 3D-Keypoints-Variation


    private VisionSubsystemListener3D visionSubsystemListener3D=null;
    private VisionSubsystemListener2D visionSubsystemListener2D=null;

    private Customize2DKeypoints customize2DkeypointsInstance=null;
    private Customize3DKeypoints customize3DkeypointsInstance=null;

    private ComplexPositionTrigger complexPositionTrigger=null;



    private ArrayList<InternalKeyPoint3D> keyPoints3D=null;  // Die zuletzt vom Canvas gemeldeten sichtbare Keypoints
    private double[] poseForKeypoints3D=null;        // Die Pose, die zu dem gemeldeten Keypoints gehört. 
                                                     // Genauer {robotPosX, robotPosY,robotAngle}
                                                     // Achtung: eigentlich bekomme ich die über newPosition(...) gemeldet, aber die ist schon wieder eine leicht andere
                                                     // Daher habe ich mir hier die Pose gemerkt, die exakt zu den Pixel-Positionen passt
                                                     // Letztendlich habe ich das aber nicht weiter verfolgt, da die Genauigkeit der 2D->3D Berechnungen nicht besser wurde
                                                     // Siehe Kommentar ** GENAUERE POSE IGNORIERT **

// Diese HashMap speichert ununterbrochene Sequenzen von "Sehen" eines Keypoints.
// Wird ein Keypoint gesehen, so wird eine Sequenz begonnen, dabei gemerkt, unter welchen Robot-Positionen der KeyPoint durchgehend gesehen wurde
// Wird ein Keypoint einmal nicht gesehen, so wird die Sequenz gelöscht
    private HashMap<Integer,ArrayList<ObservedKeyPoint2D>> sequences=new HashMap<>();  // KeyPoint3D-ID -> Liste aller durchgehender Observierungspunkt

    private long lastLineIDincrementMillis; 

    private boolean unsupportedWarung=false;  // Damit man EINMALIG ein Warnung rausgeben kann, dass man eine Methode verwendet, die auf dem echten Carbot nicht existiert
    


    public KeyPointRecognizer(Customize2DKeypoints customize2DkeypointsInstance,Customize3DKeypoints customize3DkeypointsInstance) {
        lastLineIDincrementMillis=System.currentTimeMillis();
        this.customize2DkeypointsInstance=customize2DkeypointsInstance;
        this.customize3DkeypointsInstance=customize3DkeypointsInstance;
    }


    public void setComplexPositionTrigger(ComplexPositionTrigger complexPositionTrigger) {
        this.complexPositionTrigger=complexPositionTrigger;
    }


// Nur für KeypointPackage 3D: Zufallsfehler (wenn über -v1 gewünscht)
// Gauß-Verteilung mit gegebener Standardabweichung, max. aber 3*Stdabw
    private double getRandom3Ddeviation() {
        double r=KEYPOINT3DVSTDDEV*random.nextGaussian();
        if (r>=3*KEYPOINT3DVSTDDEV)
            return 3*KEYPOINT3DVSTDDEV;
        if (r<=-3*KEYPOINT3DVSTDDEV)
            return -3*KEYPOINT3DVSTDDEV;
        return r;
    }



/**
* Change the timing configuration.
* @param timingType one of TRIGGERED, EQUIDISTANT, SYNCHRONIZED
* @param deltaT for EQUIDISTANT, SYNCHRONIZED: the cycle time in ms. For TRIGGERED must be -1
*/
    public void setTiming(int timingType,long deltaT) {
        if (complexPositionTrigger==null)
            throw new IllegalStateException("setComplexPositionTrigger must be called prior to setTiming");
        complexPositionTrigger.setTiming(timingType,deltaT);
    }


/**
* Compute a set of viewable keypoints. Keypoints must be inside the current camera view and not hidden by walls.
* @param viewAreaKeyPoints the plane projection of the view area
* @param posX current robot position x (currently ignored)
* @param posY current robot position y (currently ignored)
* @param angle current robot orientation (currently ignored)
* @param cameraX the current camera position x
* @param cameraY the current camera position y
* @param currentSina  the current camera orientation, sin(a) of it
* @param currentCosa  the current camera orientation, cos(a) of it
* @param wallSlices  the slices that may hide the keypoints
* @return all viewable keypoints
*/
    public ArrayList<InternalKeyPoint3D> computeKeyPoints3D(Geometry viewAreaKeyPoints,
                                                            double posX,double posY,double angle,
                                                            double cameraX,double cameraY,double currentSina,double currentCosa,
                                                            ArrayList<Slice> wallSlices) {

        long currentMS=System.currentTimeMillis();
        if (currentMS-lastLineIDincrementMillis>=MS_FOR_NEW_KEYPOINTS) {
            Environment.incKeyPointLineIDs();
            lastLineIDincrementMillis=currentMS;
        }

        // Alle KeyPoints im Sichtkegel beschaffen
        ArrayList<InternalKeyPoint3D> keyPointsInView3D=Environment.getKeyPointsInView(viewAreaKeyPoints);

        // Die Kamera-Projektion braucht man, um die Keypoints zu projizieren und festzustellen, ob die auf dem Kamera-Bild liegen (der Sichtkegel ist ja nur eine grobe Annährerung)
        Projection projectionForKeyPoints=new Projection(Robot.cameraSizeX,Robot.cameraSizeY,cameraX,cameraY,currentSina,currentCosa);

        // KeyPoints aussortieren, die nicht auf dem Kamera-Bild erscheinen
        for (int i=keyPointsInView3D.size()-1;i>=0;i--) {
            InternalKeyPoint3D kp=keyPointsInView3D.get(i);
            int[] projection2d=projectionForKeyPoints.project2D(kp.getPosition());
            kp.pixelX=projection2d[0];
            kp.pixelY=projection2d[1];

            if (kp.pixelX>Robot.cameraSizeX/2 ||
                kp.pixelX<-Robot.cameraSizeX/2 || 
                kp.pixelY>=Robot.cameraSizeY/2 ||
                kp.pixelY<-Robot.cameraSizeY/2) {
                keyPointsInView3D.remove(i);
            }
        }

        // Die Keypoints anhand der WallSlices aussortieren!
        for (int i=keyPointsInView3D.size()-1;i>=0;i--) {
            double[] keypoint=keyPointsInView3D.get(i).getPosition();
            for (int j=wallSlices.size()-1;j>=0;j--) {
                Slice slice=wallSlices.get(j);
                if (slice.intersects(cameraX,cameraY,Robot.camHeight,keypoint[0],keypoint[1],keypoint[2])) {
                    keyPointsInView3D.remove(i);
                    break;
                }
            }
        }

        enterKeyPoints3D(keyPointsInView3D,new double[]{posX,posY,angle});
        return keyPointsInView3D;
    }


/**
* Pass the last set of keypoints to this object. Multiple sets of keypoints maybe passed, before the actual computation is triggered
* by a new position.
* @param keyPoints3D a list of keypoints currently in view by the robot camers
* @param pose array of three with {x, y, angle} of the robot (currently ignored)
*/
    private synchronized void enterKeyPoints3D(ArrayList<InternalKeyPoint3D> keyPoints3D,double[] pose) {   // synchronized ist gefährlich, da diese Methode aus dem Canvas.repaint aufgerufen wird, allerdings muss ich den Zugriff auf keyPoints3D steuern
        this.keyPoints3D=keyPoints3D;
        this.poseForKeypoints3D=pose;
    }


// Letzte Meldung der Keypoints zurückgeben, damit eine Kopie angelegt werden kann
// Zugriff über Methode, damit synchronized
    private synchronized ArrayList<InternalKeyPoint3D> getCurrentKeyPoints3D(
//             double[] pose
            ) {  
//        pose[0]=poseForKeypoints3D[0];
//        pose[1]=poseForKeypoints3D[1];
//        pose[2]=poseForKeypoints3D[2];
        return keyPoints3D;
    }


/**
* The motion subsystem reports a new position.
* As a consequence, the keypoint recognizer computes a set of new keypoints and reports them to formlery registered keypoints listeners (3D <i>and</i> 2D), e.g. to the robot controller.
* @param receiveTimeMillis system time when receiving the position measurement or when estimating the position
* @param type position measurement type, see constants
* @param posX current X cm
* @param posY current y cm
* @param angle orientation degrees
* @throws Exception whatever happens
*/
    public void newPosition(long receiveTimeMillis,
                            int type,
                            double posX,double posY,double angle
                           ) throws Exception {

        if (poseForKeypoints3D==null)   // Es kann am Anfang passieren, dass noch nie enterKeyPoints3D ausgeführt wurde
            return;

        ArrayList<InternalKeyPoint3D> keyPoints3DlastCopy=getCurrentKeyPoints3D();     // Lokale Kopie anlegen, damit asynchron neue Keypoints geschrieben werden können

// ** GENAUERE POSE IGNORIERT **
//        double[] poseForKeyPoints=new double[3];
//        ArrayList<InternalKeyPoint3D> keyPoints3DlastCopy=getCurrentKeyPoints3D(poseForKeyPoints);
//        posX=poseForKeyPoints[0];   // Achtung Hier Testweise die Pose des Keypoints holen, da diese genauer zu den Pixel-Positionen in den Keypoints passt
//        posY=poseForKeyPoints[1];   // Das hat aber keine genaueren 2D-Keypoints gebracht
//        angle=poseForKeyPoints[2];


 // Menge alle Keypoint-IDs machen
        HashSet<Integer> occuredKeypoints=new HashSet<>(); 
        for (InternalKeyPoint3D keypoint: keyPoints3DlastCopy) {
            occuredKeypoints.add(keypoint.internalID);
        }

 // Alle Sequenzen löschen, die nicht mehr in der neuen Messung vorkommen
        ArrayList<Integer> toRemove=new ArrayList<>(sequences.size());  // Leider kann man nicht gleichzeitig durchlaufen und löschen, daher Liste aufbauen, was später gelöscht werden soll
        for (Integer keypointID:sequences.keySet()) {
            if (!occuredKeypoints.contains(keypointID))
                toRemove.add(keypointID);
        }
        for (Integer rmv:toRemove)  // Alle vorher gemerkten löschen
            sequences.remove(rmv);


 // Jetzt alle Messungen mit den gespeicherten Sequenzen verrechnen
 // Dabei das Resultat 3D aufbauen
 // - kommt diese ID vor, Sequenz erweitern
 // - kommt diese ID nicht vor, eine neue Sequenz eintragen

        KeyPointPackage3D keyPointPackage3D=new KeyPointPackage3D();  // Hier kommt das Resultat für den RobotController

        keyPointPackage3D.observationPosX=DCMotorController.currentPosX;  // Achtung: hier soll nicht die gemeldete Postion rein, sondern die "echte". Es wird also so getan, als ob ein SLAM-Mechanismus die interne Position korrigiert hat
        keyPointPackage3D.observationPosY=DCMotorController.currentPosY;
        keyPointPackage3D.observationAngle=DCMotorController.currentAngle;
        keyPointPackage3D.positionType=type;
        
        keyPointPackage3D.mssPosX=posX;  // Das ist die Position, die vom MSS gemeldet wurde (also ungenau)
        keyPointPackage3D.mssPosY=posY;
        keyPointPackage3D.mssAngle=angle;

        // Alle Keypoints auf observed=false setzen
        Environment.setKeyPointsUnobserved();

        for (InternalKeyPoint3D keypoint: keyPoints3DlastCopy) {

            ArrayList<ObservedKeyPoint2D> seq=sequences.get(keypoint.internalID);
            if (seq==null) {           // Diese Sequenz gab es noch nicht
                seq=new ArrayList<ObservedKeyPoint2D>();   // Sequenz eintragen: 1. und letzter Aufnahmepunkt ist die aktuelle Position

                ObservedKeyPoint2D newObkp=new ObservedKeyPoint2D();
                newObkp.lineID=-1;
                newObkp.observationPosX=posX;
                newObkp.observationPosY=posY;
                newObkp.observationAngle=angle;
                newObkp.pixelX=keypoint.pixelX+Robot.cam_cx;
                newObkp.pixelY=keypoint.pixelY+Robot.cam_cy;
                newObkp.undistortedNormalizedX=keypoint.pixelX/Robot.cam_fx;
                newObkp.undistortedNormalizedY=-keypoint.pixelY/Robot.cam_fy;

                seq.add(newObkp);

                sequences.put(keypoint.internalID,seq);
            }
            else {                     // Diese Sequenz gab es schon
                                       // Suche von allen bisherigen Punkten denjenige aus, der am weitesten vom letzten Punkt entfernt ist
                                       // Das ist der Observation-Punkt 1, der andere ist der aktuelle
                                       // Da ich die 3D-Position hier einfach "auslesen" kann, brauche ich eigentlich den ersten Punkt nicht
                                       // Für den RobotController muss ich aber so tun, als ob die 3D-Position aus zwei Einzelmessungen berechnet wurde
                double maxSqrDist=0.0;
                ObservedKeyPoint2D maxKpo=null;
                for ( ObservedKeyPoint2D kpo:seq) {
                    double sqrDist=(kpo.observationPosX-posX)*(kpo.observationPosX-posX)+(kpo.observationPosY-posY)*(kpo.observationPosY-posY);
                    if (sqrDist>maxSqrDist) {
                        maxSqrDist=sqrDist;
                        maxKpo=kpo;
                    }
                }

                ObservedKeyPoint2D newObkp=new ObservedKeyPoint2D();
                newObkp.lineID=-1;
                newObkp.observationPosX=posX;
                newObkp.observationPosY=posY;
                newObkp.observationAngle=angle;
                newObkp.pixelX=keypoint.pixelX+Robot.cam_cx;
                newObkp.pixelY=keypoint.pixelY+Robot.cam_cy;
                newObkp.undistortedNormalizedX=keypoint.pixelX/Robot.cam_fx;
                newObkp.undistortedNormalizedY=-keypoint.pixelY/Robot.cam_fy;

                seq.add(newObkp);  // Hinzufügen der letzten Position

                if (maxSqrDist>MIN_SQR_DIST_FOR_VIEW) {          // Die aktuelle Position ist weit genug von der entferntesten Aufnahmeposition weg

                    keypoint.observed=true;  // Merken, dass dieser Keypoint auf zwei Aufnahmen erschienen ist (das brauche ich für das Malen der Keypoints)

                    ObservedKeyPoint3D obskp=new ObservedKeyPoint3D();   // Einen Resultat-Keypoint (3D) für den Robotcontroller bauen
                    obskp.x=keypoint.x;
                    obskp.y=keypoint.y;
                    obskp.z=keypoint.z;

                    if (KEYPOINT3DVSTDDEV>0.0d) {
                        obskp.x+=getRandom3Ddeviation();
                        obskp.y+=getRandom3Ddeviation();
                        obskp.z+=getRandom3Ddeviation();
                    }

                    obskp.observationPos1X=maxKpo.observationPosX;
                    obskp.observationPos1Y=maxKpo.observationPosY;
                    obskp.observationAngle1=maxKpo.observationAngle;
                    obskp.pixel1X=maxKpo.pixelX;
                    obskp.pixel1Y=maxKpo.pixelY;
                    obskp.undistortedNormalized1X=maxKpo.undistortedNormalizedX;
                    obskp.undistortedNormalized1Y=maxKpo.undistortedNormalizedY;
                    
                    obskp.observationPos2X=newObkp.observationPosX;
                    obskp.observationPos2Y=newObkp.observationPosY;
                    obskp.observationAngle2=newObkp.observationAngle;
                    obskp.pixel2X=newObkp.pixelX;
                    obskp.pixel2Y=newObkp.pixelY;
                    obskp.undistortedNormalized2X=newObkp.undistortedNormalizedX;
                    obskp.undistortedNormalized2Y=newObkp.undistortedNormalizedY;

                    keyPointPackage3D.observedKeyPoints.add(obskp);
                }
            }
        }

        if (visionSubsystemListener3D!=null) {

            if (customize3DkeypointsInstance!=null) {
                try {
                    customize3DkeypointsInstance.customize(keyPointPackage3D);
                }
                catch (Exception e) {
                    System.out.println("Exception during 3D Keypoint customization "+e);
                    e.printStackTrace();
                }
            }

            visionSubsystemListener3D.observedKeypoints3D(keyPointPackage3D);  // An Listener übergeben       
        }


 // Jetzt alle Messungen auch noch in 2D melden
 // Aber nur, wenn es überhaupt einen Listener gibt
        if (visionSubsystemListener2D!=null) {
            KeyPointPackage2D keyPointPackage2D=new KeyPointPackage2D();  // Hier kommt das Resultat für den RobotController

            keyPointPackage2D.observationPosX=posX;  // Das ist die Position, die vom MSS gemeldet wurde (also ungenau)
            keyPointPackage2D.observationPosY=posY;
            keyPointPackage2D.observationAngle=angle;
            keyPointPackage2D.positionType=type;

            for (InternalKeyPoint3D keypoint: keyPoints3DlastCopy) {
                ObservedKeyPoint2D obskp=new ObservedKeyPoint2D();   // Einen Resultat-Keypoint (2D) für den Robotcontroller bauen

                obskp.lineID=keypoint.lineID; 

                obskp.observationPosX=posX;
                obskp.observationPosY=posY;
                obskp.observationAngle=angle;

                obskp.pixelX=keypoint.pixelX+Robot.cam_cx;
                obskp.pixelY=keypoint.pixelY+Robot.cam_cy;
                
                obskp.undistortedNormalizedX=keypoint.pixelX/Robot.cam_fx;
                obskp.undistortedNormalizedY=-keypoint.pixelY/Robot.cam_fy;

                keyPointPackage2D.observedKeyPoints.add(obskp);
            }

            if (customize2DkeypointsInstance!=null) {
                try {
                    customize2DkeypointsInstance.customize(keyPointPackage2D);
                }
                catch (Exception e) {
                    System.out.println("Exception during 2D Keypoint customization "+e);
                    e.printStackTrace();
                }
            }
            visionSubsystemListener2D.observedKeypoints2D(keyPointPackage2D);  // An Listener übergeben       
        }
    }


/**
* Required to implement the simulated vision subsystem. Register a listener to asynchronous keypoint packages from the vision subsystem.
* @param listener object that is informed when an asynchronous keypoint package appears from the vision subsystem.
* @throws UnsupportedOperationException if the vision subsystem is not able to produce 3D keypoints
*/
    public void registerVisionListener3D(VisionSubsystemListener3D listener) throws UnsupportedOperationException {
        if (visionSubsystemListener3D!=null)
            throw new RuntimeException("This Vision Subsystem can only register one listener");

        visionSubsystemListener3D=listener;
    }


/**
* Register a listener to asynchronous keypoint packages (2D) from the vision subsystem.
* @param listener object that is informed when an asynchronous keypoint package appears from the vision subsystem.
* @throws UnsupportedOperationException if the vision subsystem is not able to produce 2D keypoints
*/
    public void registerVisionListener2D(VisionSubsystemListener2D listener) throws UnsupportedOperationException {
        if (visionSubsystemListener2D!=null)
            throw new RuntimeException("This Vision Subsystem can only register one listener");
    
        visionSubsystemListener2D=listener;
    }


/**
* Informs the vision subsystem the reset all data of formerly found keypoints (2D). As a result, it tries to find new keypoints
* the next time. Even if these keypoints belong to formlery found objects, they are considered as 'new' &ndash; in particular, their 
* <i>lineID</i> gets a new value.
*/
    public void findNewKeypoints2D() {
        lastLineIDincrementMillis=0;   // Einfach den Zeitpunkt künstlich auf "Anfang" setzen, damit werden beim nächsten Mal automatisch neue LineIDs vergeben
    }


/**
* <p>Returns all (visible and invisible) keypoints (3D) of the entrire environment. This mechanism will definitly <i>not</i> be available on a real robot as
* it requires to capture the entire environment (i.e. &quot;the whole world&quot;).</p>
* This is a useful function to, e.g., test navigation algorithms. As the entire map of all obstacles is instantly available (without driving around to view all),
* a navigation from position to position can immediatly be tested.
* <i>Important:</i> as the obeserved keypoints are not actually observed by the (simulated) camera, the all entries related to camera views (i.e. observation position, pixels, undistored normalized pixels),
* are <i>invalid</i>. Only the actual position (x, y, z) contains useful values.
* @return keypoints (3D)
* @throws UnsupportedOperationException on all real Carbots
*/
    public KeyPointPackage3D getAllKeypoints3D() throws UnsupportedOperationException {

        if (!unsupportedWarung) {
            System.out.println("Caution! getAllKeypoints3D() will NOT be available on a real Carbot");
            System.out.println(" - use this function for testing only");
            unsupportedWarung=true;
        }

        KeyPointPackage3D keyPointPackage3D=new KeyPointPackage3D();  // Hier kommt das Resultat für den RobotController

        keyPointPackage3D.observationPosX=DCMotorController.currentPosX;  // Achtung: hier soll nicht die gemeldete Postion rein, sondern die "echte". Es wird also so getan, als ob ein SLAM-Mechanismus die interne Position korrigiert hat
        keyPointPackage3D.observationPosY=DCMotorController.currentPosY;
        keyPointPackage3D.observationAngle=DCMotorController.currentAngle;

        for (InternalKeyPoint3D keypoint: Environment.keyPoints3D) {

            ObservedKeyPoint3D obskp=new ObservedKeyPoint3D();   // Einen Resultat-Keypoint (3D) für den Robotcontroller bauen
            obskp.x=keypoint.x;
            obskp.y=keypoint.y;
            obskp.z=keypoint.z;

            obskp.observationPos1X=keyPointPackage3D.observationPosX;
            obskp.observationPos1Y=keyPointPackage3D.observationPosY;
            obskp.observationAngle1=keyPointPackage3D.observationAngle;
            obskp.pixel1X=Integer.MIN_VALUE;
            obskp.pixel1Y=Integer.MIN_VALUE;
            obskp.undistortedNormalized1X=Double.NaN;
            obskp.undistortedNormalized1Y=Double.NaN;
                    
            obskp.observationPos2X=keyPointPackage3D.observationPosX;
            obskp.observationPos2Y=keyPointPackage3D.observationPosY;
            obskp.observationAngle2=keyPointPackage3D.observationAngle;
            obskp.pixel2X=Integer.MIN_VALUE;
            obskp.pixel2Y=Integer.MIN_VALUE;
            obskp.undistortedNormalized2X=Double.NaN;
            obskp.undistortedNormalized2Y=Double.NaN;

            keyPointPackage3D.observedKeyPoints.add(obskp);
        }

        return keyPointPackage3D;
    }

}