package sim.environment;


import java.io.File;
import java.io.FileInputStream;
import java.io.DataInputStream;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.IOException;

import java.util.ArrayList;
import java.util.Random;


import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.operation.distance.DistanceOp;

import jr.motion.DCMotorController;
import tetrix.Tetrix;

import sim.SimCanvas;
import sim.keypoints.InternalKeyPoint3D;
import sim.geom.CMGeom;
import robotinterface.Robot;

/**
* Central point the access the simulated environment.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Environment {

    private final static double MIN_INITIAL_DIST_TO_WALL=45;            // Initialer Mindestabstand zu irgendeiner Wand
    private final static double US_RANGE=Robot.usSensingDistance;       // Sichtweite des US-Sensors
    private final static double US_ANGLEWIDTH=US_RANGE*Math.tan(Robot.usSensingAngle*Robot.PIDIV180/2);      // Am Ende der Sichtweite des US-Sensors kann man +- diesen Wert rechts und links die Entfernung erkennen

    private final static double US_RANGE_FOR_DISPLAY=150.0d;  // Diese Entfernung oder n�her, dann wird eine Distanzlinie f�r Ultaschall gemalt                                                              

    private final static double PIDIV180=Math.PI/180.0d;

    private final static int DEGREE_PER_APPROX_CIRCLE_SEGMENT=20;      // Grad bei einem Bogen, der durch eine Linie verbunden wird
    private final static int DEGREE_PER_APPROX_CIRCLE_SEGMENT_BOW=20;  // Grad bei einem Bogen, der durch eine Linie verbunden wird

    public static boolean LINEIDS_NOREFRESH=false;                     // Wunsch von Wolfhagen: Keypoints LineID sollen immer dieselben bleiben, auch wenn man sie verloren hatte - brauhte er f�r Tests

    private static GeometryFactory geomfact=new GeometryFactory();
    private BufferedReader br=null;
    private int brLine=0;

    public static ArrayList<Slick> slicks=new ArrayList<>();

    public static ArrayList<Wall> walls=new ArrayList<>();
    public static Geometry wallsJTSGeom=null;                  // Alle W�nde
    public static Geometry obstaclesJTSGeom=null;              // Alle Hindernisse
    public static Geometry boxesJTSGeom=null;                  // Alle Boxes
    public static Geometry flatsJTSGeom=null;                  // Alle Flats
    public static Geometry wallsAndObstaclesJTSGeom=null;      // Alle W�nde und und Hindernisse
    public static Geometry allwallsJTSGeom=null;               // Alle W�nde, Hinderniss und "Nur-Taktile" (also alles)


    public static Geometry allshaddowsJTSGeom=null;            // Alle Schatten


    public static ArrayList<InternalKeyPoint3D> keyPoints3D=new ArrayList<>();  // Keypoints zur Umgebung


    public static double[] lastTaktileHitPoint=null;   // Punkt, wo zuletzt der Taktile Sensor getroffen wurde (oder null, wenn Taktil nicht ausgel�st)
    public static double[][] lastUSHit=null;           // Linie der letzte US-Messung (oder null, wenn zu weit weg)

    private final static Random random=new Random(123);          // Zufallszahl f�r Area Keypoints

    private static double MIN_DIST_KEYPOINTS_POS=   50.0d;    // Minimaler Abstand zweier Keypoints auf einer H�he 
    private static double MIN_DIST_KEYPOINTS_AREA= 100.0d;    // Minimaler Abstand zweier Keypoints von Areas
    private static int MAX_AREA_KEYPOINTS=5;                  // Maximale Anzahl von Keypoints in einer Fl�che


    private double initialRobotX=0.0d;
    private double initialRobotY=0.0d;
    private double initialRobotAngle=0.0d;

    public static String robotControllerConfig=null;          // Config f�r Robot-Controller (dasselbe wie �ber Cmd-Line, nur im Envir-File
    public boolean hasCamera=false;
    public boolean hasKeypointsConfig=false;


    public void processCAMERAdirective(String strLine,ArrayList<String> args) {
        throwIllegalArgsCount(strLine,args,3);
        Robot.cameraSizeX=intFromArg(args,1);
        Robot.cameraSizeY=intFromArg(args,2);
                 
        Robot.cam_fx=Robot.cameraSizeX/2/Robot.tanCamHorViewAngleHalf;
        Robot.cam_fy=Robot.cameraSizeY/2/Robot.tanCamVerViewAngleHalf;
        Robot.cam_cx=Robot.cameraSizeX/2;                       
        Robot.cam_cy=Robot.cameraSizeY/2;                       

        throwIllegalDouble(Robot.cameraSizeX,Robot.cameraSizeX>=100,"cameraSizeX>=100");
        throwIllegalDouble(Robot.cameraSizeY,Robot.cameraSizeY>=100,"cameraSizeY>=100");
        hasCamera=true;
    }


    public void processKEYPOINTSdirective(String strLine,ArrayList<String> args) {
        throwIllegalArgsCount(strLine,args,5);
        MIN_DIST_KEYPOINTS_POS=doubleFromArg(args,1);
        MIN_DIST_KEYPOINTS_AREA=doubleFromArg(args,2);
        MAX_AREA_KEYPOINTS=intFromArg(args,3);
        SimCanvas.MAX_VIEW_DIST_KEYPOINTS=doubleFromArg(args,4);
        hasKeypointsConfig=true;
    }



/**
* Create an environment from file.
* @param filename file name
* @throws IOException if file is not accessible
* @throws IllegalArgumentException if file cannot be parsed
*/
     public Environment(String filename) throws IOException,IllegalArgumentException {

         if (filename==null)
             return;

         br=new BufferedReader(new InputStreamReader(new DataInputStream(new FileInputStream(new File(filename)))));

         boolean hasRobot=false;
         boolean hasRobotController=false;

         keyPoints3D=new ArrayList<>(200);

         String strLine;
         while ((strLine=nextLine()) != null) {
             strLine=removeComments(strLine);
             if (strLine.equals(""))
                 continue;
             ArrayList<String> args=args(strLine);
             if (args.get(0).equals("ROBOT")) {
                 if (hasRobot)
                     throw new IllegalArgumentException("Multiple ROBOT directives [Line "+brLine+"]");
                 throwIllegalArgsCount(strLine,args,4);
                 initialRobotX=doubleFromArg(args,1);
                 initialRobotY=doubleFromArg(args,2);
                 initialRobotAngle=doubleFromArg(args,3);
                 hasRobot=true;
             }
             else if (args.get(0).equals("ROBOTCONTROLLER")) {
                 if (hasRobotController)
                     throw new IllegalArgumentException("Multiple ROBOTCONTROLLER directives [Line "+brLine+"]");
                     
                 throwIllegalArgsCount(strLine,args,2);
                 robotControllerConfig=args.get(1);
                     
                 hasRobotController=true;
             }
             else if (args.get(0).equals("KEYPOINTS")) {
                 if (hasKeypointsConfig)
                     throw new IllegalArgumentException("Multiple KEYPOINTS directives [Line "+brLine+"]");
                 processKEYPOINTSdirective(strLine,args);
             }
             else if (args.get(0).equals("CAMERA")) {
                 if (hasCamera)
                     throw new IllegalArgumentException("Multiple CAMERA directives [Line "+brLine+"]");

                 processCAMERAdirective(strLine,args);               
             }
             else if (args.get(0).equals("KEYPOINT")) {
                 throwIllegalArgsCount(strLine,args,4);
                 double kpx=doubleFromArg(args,1);
                 double kpy=doubleFromArg(args,2);
                 double kpz=doubleFromArg(args,3);
                 keyPoints3D.add(new InternalKeyPoint3D(kpx,kpy,kpz));

             }
             else if (args.get(0).equals("SLICK")) {
                 Slick slick=readSlick(strLine,args);
                 slicks.add(slick);
             }
             else if (args.get(0).equals("WALL")) {
                 Wall wall=readWall(strLine,args,Wall.REAL_WALL,"ENDWALL");
                 walls.add(wall);
             }
             else if (args.get(0).equals("OBSTACLE")) {
                 Wall wall=readWall(strLine,args,Wall.OBSTACLE,"ENDOBSTACLE");
                 walls.add(wall);
             }
             else if (args.get(0).equals("BOX")) {
                 Wall wall=readWall(strLine,args,Wall.BOX,"ENDBOX");
                 walls.add(wall);
             }
             else if (args.get(0).equals("FLAT")) {
                 Wall wall=readWall(strLine,args,Wall.FLAT,"ENDFLAT");
                 walls.add(wall);
             }
             else 
                 throw new IllegalArgumentException("Don't understand directive '"+strLine+"' [Line "+brLine+"]");
         }

         Geometry wallsKP_JTSGeom=null;                  // Alle W�nde MIT Keypoints
         Geometry obstaclesKP_JTSGeom=null;              // Alle Hindernisse MIT Keypoints
         Geometry boxesKP_JTSGeom=null;                  // Alle Boxes MIT Keypoints
         Geometry flatsKP_JTSGeom=null;                  // Alle Flats MIT Keypoints


         // Alle Walls zu einer JTS-Geometry zusammenbauen
         if (walls.size()>0) {
             for (int i=0;i<walls.size();i++) {
                 Wall wall=walls.get(i);
                 if (wall.type==Wall.FLAT) {
                     flatsJTSGeom=unionGeom(flatsJTSGeom,wall.jtsGeom);
                     if (wall.hasKeypoints)
                        flatsKP_JTSGeom=unionGeom(flatsKP_JTSGeom,wall.jtsGeom);
                 }
                 
                 else if (wall.type==Wall.BOX) {
                     boxesJTSGeom=unionGeom(boxesJTSGeom,wall.jtsGeom);
                     if (wall.hasKeypoints)
                         boxesKP_JTSGeom=unionGeom(boxesKP_JTSGeom,wall.jtsGeom);
                 }                 
                 
                 else if (wall.type==Wall.OBSTACLE) {
                     obstaclesJTSGeom=unionGeom(obstaclesJTSGeom,wall.jtsGeom);
                     if (wall.hasKeypoints)
                         obstaclesKP_JTSGeom=unionGeom(obstaclesKP_JTSGeom,wall.jtsGeom);
                 }
                 
                 else {  // WALL
                     wallsJTSGeom=unionGeom(wallsJTSGeom,wall.jtsGeom);
                     if (wall.hasKeypoints)
                         wallsKP_JTSGeom=unionGeom(wallsKP_JTSGeom,wall.jtsGeom);
                 }

                 if (wall.type==Wall.BOX || wall.type==Wall.OBSTACLE || wall.type==Wall.REAL_WALL) {
                     wallsAndObstaclesJTSGeom=unionGeom(wallsAndObstaclesJTSGeom,wall.jtsGeom);
                 }
                 allwallsJTSGeom=unionGeom(allwallsJTSGeom,wall.jtsGeom);

                 if (wall.jtsShadowGeom!=null) {
                     allshaddowsJTSGeom=unionGeom(allshaddowsJTSGeom,wall.jtsShadowGeom);
                 }
             }
         }
         
         if (flatsKP_JTSGeom!=null)
             keyPoints3D.addAll(getKeyPoints(flatsKP_JTSGeom,0.0d));
             
         if (boxesKP_JTSGeom!=null)
             keyPoints3D.addAll(getKeyPoints(boxesKP_JTSGeom,Wall.HEIGHTS[Wall.BOX]));
             
         if (obstaclesKP_JTSGeom!=null)
             keyPoints3D.addAll(getKeyPoints(obstaclesKP_JTSGeom,Wall.HEIGHTS[Wall.OBSTACLE]));
             
         if (wallsKP_JTSGeom!=null)
             keyPoints3D.addAll(getKeyPoints(wallsKP_JTSGeom,Wall.HEIGHTS[Wall.REAL_WALL]));


         // Checken der Entfernung der Robot-Position von den n�hesten Wandposition
         Point robotXY=geomfact.createPoint(new Coordinate(initialRobotX,initialRobotY));
         double distToWall=distTowall(robotXY);
         if (distToWall<MIN_INITIAL_DIST_TO_WALL)
             throw new IllegalArgumentException("Robot's minimum distance to any wall should be at least "+MIN_INITIAL_DIST_TO_WALL+"cm but is "+distToWall+"cm [Line "+brLine+"]");
//         System.out.println("WARNING: Robot's minimum distance to any wall should be at least "+MIN_INITIAL_DIST_TO_WALL+"cm but is "+distToWall+"cm [Line "+brLine+"]");

         setInitialPosition();
     }
     
     
     private static Geometry unionGeom(Geometry geom,Geometry wallGeom) {
        if (geom==null) 
            return wallGeom;
        else
            return geom.union(wallGeom);
     }

/**
* Set the robot to the initial position defined by the environment file (or to 0,0,0).
*/
     public void setInitialPosition() {
         DCMotorController.currentPosX=initialRobotX;
         DCMotorController.currentPosY=initialRobotY;
         DCMotorController.currentAngle=initialRobotAngle;
         Tetrix.currentPosX=initialRobotX;
         Tetrix.currentPosY=initialRobotY;
         Tetrix.currentAngle=initialRobotAngle;
     }


// Berechne alle Keypoints einer Region mit der H�he. Es werden drei S�tze erzeugt (alle landen in einer Liste)
// 1: f�r die Bodenlinie
// 2: f�r die Linie height (nur wenn height >1)
// 3: Fl�chenpunkte
     private static ArrayList<InternalKeyPoint3D> getKeyPoints(Geometry geom,double height) {
         ArrayList<ArrayList<double[][]>> allPos=CMGeom.cmFromJTSPolyGeometry(geom);

         ArrayList<InternalKeyPoint3D> keypoints=getKeyPointsForPositions(allPos,0.0d);   // Die Keypoints der Bodenlinie
         if (height>=1.0d)
             keypoints.addAll(getKeyPointsForPositions(allPos,height));         // Die Keypoints der Bodenlinie

         keypoints.addAll(getKeyPointsForAreas(allPos,height));                 // Keypoints in der Fl�che
         return keypoints;     
     }


// Keypoints f�r Bodenlinie oder obere Kante
     private static ArrayList<InternalKeyPoint3D> getKeyPointsForPositions(ArrayList<ArrayList<double[][]>> allPos,double height) {
         ArrayList<InternalKeyPoint3D> keypoints=new ArrayList<>(200);

         for (ArrayList<double[][]> poly:allPos) {

             double minX=1e10;
             double minY=1e10;
             double maxX=-1e10;
             double maxY=-1e10;
             for (double[] shellPoint:poly.get(0)) {
                 minX=Math.min(minX,shellPoint[0]);
                 maxX=Math.max(maxX,shellPoint[0]);
                 minY=Math.min(minY,shellPoint[1]);
                 maxY=Math.max(maxY,shellPoint[1]);
             }

             boolean hasMinX=false;
             boolean hasMaxX=false;
             boolean hasMinY=false;
             boolean hasMaxY=false;

             for (double[][] shellOrHole: poly) {
                 InternalKeyPoint3D lastKeyPoint=null;
                 for (int i=0;i<shellOrHole.length;i++) {
                     InternalKeyPoint3D newKeyPoint=new InternalKeyPoint3D(
                         shellOrHole[i][0],
                         shellOrHole[i][1],
                         height
                       );
                     if (lastKeyPoint==null) {
                         keypoints.add(newKeyPoint);
                         lastKeyPoint=newKeyPoint;
                     }
                     else {

                         if (!hasMinX && shellOrHole[i][0]==minX) {
                             keypoints.add(newKeyPoint);
                             lastKeyPoint=newKeyPoint;
                             hasMinX=true;
                         }
                         else if (!hasMaxX && shellOrHole[i][0]==maxX) {
                             keypoints.add(newKeyPoint);
                             lastKeyPoint=newKeyPoint;
                             hasMaxX=true;
                         }
                         else if (!hasMinY && shellOrHole[i][1]==minY) {
                             keypoints.add(newKeyPoint);
                             lastKeyPoint=newKeyPoint;
                             hasMinY=true;
                         }
                         else if (!hasMaxY && shellOrHole[i][1]==maxY) {
                             keypoints.add(newKeyPoint);
                             lastKeyPoint=newKeyPoint;
                             hasMaxY=true;
                         }
                         else {
                             int iPred=(i-1) % shellOrHole.length;
                             int iSucc=(i+1) % shellOrHole.length;
                             double dx0=shellOrHole[i][0]-shellOrHole[iPred][0];
                             double dy0=shellOrHole[i][1]-shellOrHole[iPred][1];
                             double dx1=shellOrHole[iSucc][0]-shellOrHole[i][0];
                             double dy1=shellOrHole[iSucc][1]-shellOrHole[i][1];
                             double cosa=(dx0*dx1+dy0*dy1)/Math.sqrt(dx0*dx0+dy0*dy0)/Math.sqrt(dx1*dx1+dy1*dy1);
                             if (cosa<0.7d) {   // Spitzer Winkel: immer
                                 keypoints.add(newKeyPoint);
                                 lastKeyPoint=newKeyPoint;
                             }
                             else {
                                 double distToLast=Math.sqrt((newKeyPoint.x-lastKeyPoint.x)*(newKeyPoint.x-lastKeyPoint.x)+
                                                             (newKeyPoint.y-lastKeyPoint.y)*(newKeyPoint.y-lastKeyPoint.y));
                                 if (distToLast>=MIN_DIST_KEYPOINTS_POS) {
                                     keypoints.add(newKeyPoint);
                                     lastKeyPoint=newKeyPoint;
                                 }
                             }
                         }
                     }
                 }
             }
         }
         return keypoints;
     }


     private static ArrayList<InternalKeyPoint3D> getKeyPointsForAreas(ArrayList<ArrayList<double[][]>> allPos,double height) {
         ArrayList<InternalKeyPoint3D> keypoints=new ArrayList<>(200);

         for (ArrayList<double[][]> poly:allPos) {
             for (double[][] shellOrHole: poly) {
                 InternalKeyPoint3D lastKeyPoint=null;
                 for (int i=0;i<shellOrHole.length-1;i++) {

                     double x0=shellOrHole[i][0];
                     double y0=shellOrHole[i][1];
                     double x1=shellOrHole[i+1][0];
                     double y1=shellOrHole[i+1][1];

                     for (int j=0;j<MAX_AREA_KEYPOINTS;j++) {
                         double relXY=random.nextDouble();
                         double relZ=random.nextDouble();
                         InternalKeyPoint3D newKeyPoint=new InternalKeyPoint3D(
                               x0+(x1-x0)*relXY,
                               y0+(y1-y0)*relXY,
                               height*relZ
                           );
                         if (lastKeyPoint==null) {
                             keypoints.add(newKeyPoint);
                             lastKeyPoint=newKeyPoint;
                         }
                         else {
                             double distToLast=Math.sqrt((newKeyPoint.x-lastKeyPoint.x)*(newKeyPoint.x-lastKeyPoint.x)+
                                                         (newKeyPoint.y-lastKeyPoint.y)*(newKeyPoint.y-lastKeyPoint.y)
                                                         );

                             if (distToLast>=MIN_DIST_KEYPOINTS_AREA) {
                                 keypoints.add(newKeyPoint);
                                 lastKeyPoint=newKeyPoint;
                             }
                         }
                     }
                 }
             }
         }
         return keypoints;
     }


/**
* Set all keypoints to <i>not observed</i>.
*/
     public static void setKeyPointsUnobserved() {
         for (InternalKeyPoint3D keypoint: keyPoints3D) {
              keypoint.observed=false;
         }
     }

/**
* Increment the keypoint's lineID (used for 2D recognition), thus all lineID are &quot;fresh&quot;.
*/
     public static void incKeyPointLineIDs() {
         if (LINEIDS_NOREFRESH)   // Wunsch von Wolfhagen f�r Tests
             return;
     
         for (InternalKeyPoint3D keypoint: keyPoints3D) {
              keypoint.lineID+=+keyPoints3D.size();
         }
     }

/**
* Get all 3D keypoints that are in view (considering the view geometry).
* @param viewArea the ground projection of the area that is in view
* @return all 3D keypoints
*/
     public static ArrayList<InternalKeyPoint3D> getKeyPointsInView(Geometry viewArea) {
         ArrayList<InternalKeyPoint3D> result=new ArrayList<>(keyPoints3D.size());
         for (InternalKeyPoint3D keypoint: keyPoints3D) {
             Point point=geomfact.createPoint(new Coordinate(keypoint.x,keypoint.y));
             if (viewArea.contains(point)) {
// BRAUCHT MAN OFFENSICHTLICH NICHT                result.add(new InternalKeyPoint3D(keypoint));  // Objekt kopieren, damit man sp�ter die Pixel-Positionen setzen kann
                 result.add(keypoint);  // Objekt kopieren, damit man sp�ter die Pixel-Positionen setzen kann
             }
         }
         return result;
     }


/**
* Get the slick object for a position (cm, ground projection).
* @param cmX position to check
* @param cmY position to check
* @return a single slick object if there is any, otherwise null
*/
     public static Slick slickFor(double cmX,double cmY) {
         for (int i=slicks.size()-1;i>=0;i--) {
             Slick slick=slicks.get(i);
             if (slick.contains(cmX,cmY))
                 return slick;
         }
         return null;
     }


// Alle W�nde (auch flat)
     private double distTowall(Geometry otherGeometry) {
         if (allwallsJTSGeom==null)
             return 10000;
         Coordinate[] nearest=new DistanceOp(allwallsJTSGeom,otherGeometry).nearestPoints();
         return Math.sqrt((nearest[0].x-nearest[1].x)*(nearest[0].x-nearest[1].x)+(nearest[0].y-nearest[1].y)*(nearest[0].y-nearest[1].y));
     }


/** 
* Check if a given geometry overlaps with any real wall, obstactle or flats. This method is used to check, if the robot collides with an object.
* @param otherGeometry the geometry to check
* @return one overlapping point (cm, ground projection) or null, for no overlap
*/
     public static double[] overlappingPointWithWall(Geometry otherGeometry) {

         if (allwallsJTSGeom==null) {
             return null;
         }


         Coordinate[] nearest=new DistanceOp(allwallsJTSGeom,otherGeometry).nearestPoints();
         double sqrDist=(nearest[0].x-nearest[1].x)*(nearest[0].x-nearest[1].x)+(nearest[0].y-nearest[1].y)*(nearest[0].y-nearest[1].y);
         if (sqrDist>0.1)
             return null;
         return new double[]{nearest[1].x,nearest[1].y};
     }


/** 
* Check if the taktile sensor (modelled as line that connects two position) overlaps with any real wall, obstactle or flat. This method is used to check, if the taktile sensor indicates a collision.
* @param leftTaktileX left position of the taktile sensor (cm, ground projection)
* @param leftTaktileY left position of the taktile sensor (cm, ground projection)
* @param rightTaktileX right position of the taktile sensor (cm, ground projection)
* @param rightTaktileY right position of the taktile sensor (cm, ground projection)
* @return true if the taktile sensor indicates a collision
*/
     public static boolean hasTaktileCollision(double leftTaktileX,double leftTaktileY,double rightTaktileX,double rightTaktileY) {

         if (allwallsJTSGeom==null) {
             lastTaktileHitPoint=null;
             return false;
         }

         Coordinate[] coords=new Coordinate[2];
         coords[0]=new Coordinate(leftTaktileX,leftTaktileY);
         coords[1]=new Coordinate(rightTaktileX,rightTaktileY);
         LineString taktileLine=geomfact.createLineString(coords);

         Coordinate[] nearest=new DistanceOp(allwallsJTSGeom,taktileLine).nearestPoints();
         double dist=Math.sqrt((nearest[0].x-nearest[1].x)*(nearest[0].x-nearest[1].x)+(nearest[0].y-nearest[1].y)*(nearest[0].y-nearest[1].y));

         boolean hitResult=dist<0.1d;
         if (hitResult)
             lastTaktileHitPoint=new double[]{nearest[1].x,nearest[1].y};
         else   
             lastTaktileHitPoint=null;

         return hitResult;
     }


/**
* Computes the distance from ultrasonic sensor the any object that is detectable by ultrasonic (that is only real wall and obstacle, and <i>not</i>flat).
* @param usSensorX ultrasonic sensor position (cm, ground projection)
* @param usSensorY ultrasonic sensor position (cm, ground projection)
* @param sina sin of the robots orientation
* @param cosa cos of the robots orientation
* @return distance to an object or max. value (i.e 255) if no object is in range
*/
     public static double getUSDistance(double usSensorX,double usSensorY,double sina,double cosa) {

// Idee f�r den US-Sensor:
// Man konstruiert einen Dreieck mit +-15Grad um die Sichtrichtung, maximal 2 Meter.
// Man schneidet alle W�nde mit diesem Dreick und Sucht den n�hesten Punkt
// der Tangens von 15Grad sind 0.26795, d.h. nach 2 Metern muss man +-53.6 cm gehen
// Die Flats werden hier rausgenommen

         if (wallsAndObstaclesJTSGeom==null) {
             lastUSHit=null;
             return 255.0d;
         }

         // Sicht-Dreieck

         Coordinate[] coords=new Coordinate[4];
         coords[0]=new Coordinate(usSensorX,usSensorY);
         coords[1]=new Coordinate(usSensorX+sina*US_RANGE-cosa*US_ANGLEWIDTH,usSensorY+cosa*US_RANGE+sina*US_ANGLEWIDTH);
         coords[2]=new Coordinate(usSensorX+sina*US_RANGE+cosa*US_ANGLEWIDTH,usSensorY+cosa*US_RANGE-sina*US_ANGLEWIDTH);
         coords[3]=coords[0];

         Geometry usViewTriangle=geomfact.createPolygon(geomfact.createLinearRing(coords),new LinearRing[0]);
         Geometry viewCut=usViewTriangle.intersection(wallsAndObstaclesJTSGeom);

         double result;
         if (viewCut.isEmpty()) {
             lastUSHit=null;
             result=255.0d;
         }
         else {
             Point usSensorPoint=geomfact.createPoint(new Coordinate(usSensorX,usSensorY));
             Coordinate[] nearest=new DistanceOp(viewCut,usSensorPoint).nearestPoints();
             result=Math.sqrt((nearest[0].x-nearest[1].x)*(nearest[0].x-nearest[1].x)+(nearest[0].y-nearest[1].y)*(nearest[0].y-nearest[1].y));

             if (result< US_RANGE_FOR_DISPLAY) {  // Wenn US nah genug, den Wert speichern f�r die Anzeige merken

                 lastUSHit=new double[][]{
                     {usSensorX,usSensorY},
                     {nearest[0].x,nearest[0].y}
                 };
             }
             else
                 lastUSHit=null;
         }
         return result;

     }


     private String removeComments(String line) {
         line=line.trim();
         int pos=line.indexOf("//");
         if (pos>=0)
             return line.substring(0,pos);
         return line;
     }


     private Slick readSlick(String strLine,ArrayList<String> args) throws IOException,IllegalArgumentException {
         throwIllegalArgsCount(strLine,args,4);
         String name=args.get(1);
         double slickLeft=doubleFromArg(args,2);
         double slickRight=doubleFromArg(args,3);

         throwIllegalDouble(slickLeft,slickLeft>=1.0d,"slickLeft>=1.0");
         throwIllegalDouble(slickRight,slickRight>=1.0d,"slickRight>=1.0");
         
         ArrayList<double[]> positions=readPositions("ENDSLICK");
         return new Slick(name,slickLeft,slickRight,positions);
     }


     private Wall readWall(String strLine,ArrayList<String> args,int type,String stopWord) throws IOException,IllegalArgumentException {
         throwIllegalArgsCount(strLine,args,2);

         String keypointsStr=args.get(1);

         boolean hasKeypoints;
         if (keypointsStr.equals("KEYPOINTS"))
             hasKeypoints=true;
         else if (keypointsStr.equals("NOKEYPOINTS"))
             hasKeypoints=false;
         else
             throw new IllegalArgumentException("Keypoints tag must by 'KEYPOINTS' or 'NOKEYPOINTS' but is '"+keypointsStr+"' [Line "+brLine+"]");

         ArrayList<double[]> positions=readPositions(stopWord);
         return new Wall(type,hasKeypoints,positions);
     }

// Erzeugt zwischenpunkte bei langen Kanten
     private ArrayList<double[]> readPositions(String stopword) throws IOException,IllegalArgumentException {
         ArrayList<double[]> positions=readPositions_(stopword);
         positions.add(positions.get(0));

         ArrayList<double[]> result=new ArrayList<>();
         double[] last=positions.get(0);
         result.add(last);
         for (int i=1;i<positions.size();i++) {
              double[] next=positions.get(i);
              /*double dist=Math.sqrt((next[0]-last[0])*(next[0]-last[0])+(next[1]-last[1])*(next[1]-last[1]));
              if (dist>100) {
                   int cnt=(int)Math.round(dist/100);
                   double newDist=dist/cnt;
                   for (int j=1;j<cnt;j++) {
                        result.add(new double[] {
                            last[0]+(next[0]-last[0])*j/cnt,
                            last[1]+(next[1]-last[1])*j/cnt,
                         });
                   }
              }*/
              result.add(next);
              last=next;
         }
         result.remove(result.size()-1);
         return result;
     }


     private ArrayList<double[]> readPositions_(String stopword) throws IOException,IllegalArgumentException {
         boolean hasCoord=false;    // Coord-Paar
         boolean hasSpecial=false;  // CIRCLE, RECT
         ArrayList<double[]> positions=new ArrayList<>();
         String strLine;
         while ((strLine=nextLine()) != null) {
             strLine=removeComments(strLine);
             if (strLine.equals(""))
                 continue;
             ArrayList<String> args=args(strLine);
             if (args.get(0).equals(stopword)) {
                 throwIllegalArgsCount(strLine,args,1);
                 return positions;
             }

             if (args.get(0).equals("CIRCLE")) {
                 if (hasCoord)
                     throw new IllegalArgumentException("CIRCLE appears after coordinate list in '"+strLine+"' [Line "+brLine+"]");
                 hasSpecial=true;
                 throwIllegalArgsCount(strLine,args,4);

                 double centreX=doubleFromArg(args,1);
                 double centreY=doubleFromArg(args,2);
                 double rad=doubleFromArg(args,3);

                 for (int i=0;i<DEGREE_PER_APPROX_CIRCLE_SEGMENT;i++) { 
                     double angle=i*360/DEGREE_PER_APPROX_CIRCLE_SEGMENT*PIDIV180;
                     positions.add(new double[]{centreX+Math.cos(angle)*rad,centreY+Math.sin(angle)*rad});
                 }
             }
             else if (args.get(0).equals("RECT")) {
                 if (hasCoord)
                     throw new IllegalArgumentException("RECT appears after coordinate list in '"+strLine+"' [Line "+brLine+"]");
                 hasSpecial=true;

                 throwIllegalArgsCount(strLine,args,5);
                 double centreX=doubleFromArg(args,1);
                 double centreY=doubleFromArg(args,2);
                 double sizeX=doubleFromArg(args,3);
                 double sizeY=doubleFromArg(args,4);

                 positions.add(new double[]{centreX-sizeX/2,centreY-sizeY/2});
                 positions.add(new double[]{centreX+sizeX/2,centreY-sizeY/2});
                 positions.add(new double[]{centreX+sizeX/2,centreY+sizeY/2});
                 positions.add(new double[]{centreX-sizeX/2,centreY+sizeY/2});
             }
             else if (args.get(0).equals("BOW")) {
                 if (hasCoord)
                     throw new IllegalArgumentException("BOW appears after coordinate list in '"+strLine+"' [Line "+brLine+"]");
                 hasSpecial=true;
                 throwIllegalArgsCount(strLine,args,7);
                 double centreX=doubleFromArg(args,1);
                 double centreY=doubleFromArg(args,2);
                 double radIn=doubleFromArg(args,3);
                 double radOut=doubleFromArg(args,4);
                 double angleFrom=doubleFromArg(args,5);
                 double angleTo=doubleFromArg(args,6);

                 int cnt=(int)Math.round(Math.abs(angleFrom-angleTo)*DEGREE_PER_APPROX_CIRCLE_SEGMENT_BOW/30);

                 for (int i=0;i<=cnt;i++) { 
                     double angle=(angleFrom+(angleTo-angleFrom)*i/cnt)*PIDIV180;
                     positions.add(new double[]{centreX+Math.cos(angle)*radIn,centreY+Math.sin(angle)*radIn});
                 }

                 for (int i=cnt;i>=0;i--) { 
                     double angle=(angleFrom+(angleTo-angleFrom)*i/cnt)*PIDIV180;
                     positions.add(new double[]{centreX+Math.cos(angle)*radOut,centreY+Math.sin(angle)*radOut});
                 }

             }
             else {
                 if (hasSpecial)
                     throw new IllegalArgumentException("Coordinate appears after RECT or CIRCLE in '"+strLine+"' [Line "+brLine+"]");
                 hasCoord=true;
                 throwIllegalArgsCount(strLine,args,2);
                 double x=doubleFromArg(args,0);
                 double y=doubleFromArg(args,1);
                 positions.add(new double[]{x,y});
             }
         }
         throw new IllegalArgumentException("Eof reached without '"+stopword+"' [Line "+brLine+"]");
     }



     private String nextLine() throws IOException {
         String line=br.readLine();
         brLine++;
         if (line==null || !line.trim().startsWith("/*"))
             return line;

         line=br.readLine();
         brLine++;
         while (line!=null && !line.trim().endsWith("*/")) {
             line=br.readLine().trim();
             brLine++;
         }

         if (line!=null) {
             line=br.readLine();
             brLine++;
             if (line==null)
                 return null;
             return line.trim();
         }
         return null;

     }


     private int intFromArg(ArrayList<String> args,int index) throws IllegalArgumentException {
         try {
             return new Integer(args.get(index)).intValue();
         }
         catch (NumberFormatException e) {
             throw new IllegalArgumentException("'"+args.get(index)+"' is not an int [Line "+brLine+"]");
         }
     }


     private double doubleFromArg(ArrayList<String> args,int index) throws IllegalArgumentException {
         try {
             return new Double(args.get(index)).doubleValue();
         }
         catch (NumberFormatException e) {
             throw new IllegalArgumentException("'"+args.get(index)+"' is not a double [Line "+brLine+"]");
         }
     }


     private void throwIllegalInt(int value,boolean condition,String conditionText) throws IllegalArgumentException {
         if (!condition) {
             throw new IllegalArgumentException("An int should hold the condition '"+conditionText+"' but its value is "+value+" [Line "+brLine+"]");
         }
     }


     private void throwIllegalDouble(double value,boolean condition,String conditionText) throws IllegalArgumentException {
         if (!condition) {
             throw new IllegalArgumentException("A double should hold the condition '"+conditionText+"' but its value is "+value+" [Line "+brLine+"]");
         }
     }


     private void throwIllegalArgsCount(String strLine,ArrayList<String> args,int count) throws IllegalArgumentException {
         if (args.size()!=count) {
             throw new IllegalArgumentException("'"+strLine.trim()+"' has "+args.size()+" arguments but not "+count+" as expected [Line "+brLine+"]");
         }
     }


     public ArrayList<String> args(String line) {
         String[] split=line.split("\\s+");
         ArrayList<String> result=new ArrayList<>();
         boolean inQuote=false;

         for (String s:split) {
             if (s.startsWith("\"")) {
                 if (s.endsWith("\"")) {
                     if (s.length()==1) {  // Nur ein einzelnes "
                         if (inQuote) {
                             inQuote=false;
                         }
                         else {
                             result.add("");
                             inQuote=true;
                         }
                     }
                     else {                // Start und Ende sind "
                         if (inQuote)
                             throw new IllegalArgumentException("'"+s+"' contains opening \", but last string was not terminated [Line "+brLine+"]");
                         result.add(s.substring(1,s.length()-1));
                     }
                 }
                 else {                    // Nur �ffnendes ""
                     if (inQuote)
                         throw new IllegalArgumentException("'"+s+"' contains opening \", but last string was not terminated [Line "+brLine+"]");
                     result.add(s.substring(1)+" ");
                     inQuote=true;
                 }
             }
             else { 
                 if (s.endsWith("\"")) {   // Nur schlie�endes "
                     if (!inQuote)
                         throw new IllegalArgumentException("'"+s+"' contains closing \", but last string was not opened [Line "+brLine+"]");
                     result.set(result.size()-1,result.get(result.size()-1)+s.substring(0,s.length()-1));
                     inQuote=false;
                 }
                 else {                    // Kein String-Argument
                     if (inQuote)
                         result.set(result.size()-1,result.get(result.size()-1)+s+" ");
                     else
                         result.add(s);
                 }
             }
         }
         if (inQuote)
             throw new IllegalArgumentException("'"+line.trim()+"' has unclosed \" [Line "+brLine+"]");

         return result;
     }
}