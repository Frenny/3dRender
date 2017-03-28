package sim;

import java.util.ArrayList;
import java.util.Collections;

import javax.swing.JComponent;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.BasicStroke;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.RenderingHints;
import javax.swing.ImageIcon;
import java.awt.Image;
import java.awt.Composite;
import java.awt.AlphaComposite;

import java.awt.event.MouseMotionListener;
import java.awt.event.MouseListener;
import java.awt.event.MouseEvent;

import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.MultiPolygon;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.TopologyException;

import jr.motion.DCMotorController;
import tetrix.Tetrix;

import sim.keypoints.InternalKeyPoint3D;
import sim.environment.Environment;
import sim.environment.Wall;
import sim.environment.Slick;

import sim.geom.Camera;
import sim.geom.Projection;
import sim.geom.Geom;
import sim.geom.CMGeom;
import sim.geom.Slice;
import sim.util.SimDebugPainter;

import robotinterface.Robot;

import java.awt.image.BufferedImage;

/**
* Canvas for the map view and camera image.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class SimCanvas extends JComponent implements MouseMotionListener,MouseListener {

    private static GeometryFactory geomfact=new GeometryFactory();

    public final static String VERSION_STR="0.996";
    private final static double ZOOM_INTOUT_SCALE=1.5d;  // Zoom-Factor pro Zoom in oder out


    private final static BasicStroke strokeNormal = new BasicStroke(3.5f);
    private final static BasicStroke strokeThin = new BasicStroke(1);
    private final static BasicStroke strokeThick = new BasicStroke(5);

    private final Font gridFont=new Font("Serif",Font.BOLD,16);

    private final static Color SHADDOW_COL=new Color(50,50,50,30);
    private final static Color FLAT_COL=new Color(150,150,150);
    private final static Color TRACK_COL=new Color(180,120,20,100);
    private final static Color TYRETRACK_COL=new Color(120,60,60,50);
    private final static Color WALL_COL=new Color(150,50,50);
    private final static Color OBSTACLE_COL=new Color(50,150,50);
    private final static Color BOX_COL=new Color(100,100,100);
    private final static Color KEYPOINT_COL=new Color(255,255,0,150);
    private final static Color KEYPOINT_OBSERVED_COL=new Color(255,100,100,150);


    private final static long REPAINT_CYCLE=40;          // Refresh-Rate des Canvas (vorher 10, das hat das System aber zu stark belastet)
    private final static long COPYRIGHT_TIME=4000;       // Wie lange soll das Copyright erscheinen
    private final static double TRACK_MINSQRDIST=25;     // Mindestfahrt in cm, um einen neuen Track-Eintrag einzuf�gen (zum Quadrat)
    private final static int MAX_TRACK_ENTRIES=500;      // Maximal Anzahl der Track-Eintr�ge


    private final static double PIDIV180=Math.PI/180.0d;

    private final static double WHEEL_WIDTH=2.0d;  // Breite des Reifens in cm
    private final static double BODY_WIDTH=19.0d;  // Breite Rumpfes cm
    private final static double BODY_LENGTH_FRONT=12.0d; // L�nge des Rumpes in cm vor dem Drehpunkt
    private final static double BODY_LENGTH_BACK=21.0d;  // L�nge des Rumpes in cm hinter dem Drehpunkt

    private final static double MIN_VIEW_DIST=10;        // Sichtweite der Kamera
    private final static double MAX_VIEW_DIST=2000;
    private final static double MAX_VIEW_DIST_TRACKS=500;  // Sichtweite f�r Tracks
    public static double MAX_VIEW_DIST_KEYPOINTS=250;  // Sichtweite f�r Keypoints  (kann �ber Environment gesetzt werden)

    private final static int CAMERA_IMAGE_SIZEX=320;  // Gr��e des Fensters auf dem Bildschirm
    private final static int CAMERA_IMAGE_SIZEY=240;
    private final static int CAMERA_OFFSET_X=10;     // Abstand zur rechten Fensterkante
    private final static int CAMERA_OFFSET_Y=10;     // Abstand zur oberen Fensterkante

    public static int camLook=0;  // 0: Standard, 1: Glass, 2: Wires
    public static int camLoc=0;   // 0: Upper left, 1: Upper right, 2: Lower left, 3: Lower right 4: Centre
    public static int camSize=0;  // 0: Org, 1: 2x, 2: 3x,3 : 4x", 4: 5x, 5:max
    public static boolean showGrid=true;
    public static boolean showInternalPose=true;
    public static boolean showKeypoints=true;
    public static boolean showCamRange=true;
    public static boolean showShaddow=true;
    public static boolean showSlick=true;
    public static boolean showCollisions=true;
    public static boolean showUS=true;
    public static boolean showTrack=true;
    public static boolean showTyreTracks=true;
    private static ArrayList<double[]> track=new ArrayList<>(MAX_TRACK_ENTRIES+10);
    private static ArrayList<double[]> tyreTrackFrontLeft=new ArrayList<>(MAX_TRACK_ENTRIES+10);
    private static ArrayList<double[]> tyreTrackFrontRight=new ArrayList<>(MAX_TRACK_ENTRIES+10);
    private static ArrayList<double[]> tyreTrackBackLeft=new ArrayList<>(MAX_TRACK_ENTRIES+10);
    private static ArrayList<double[]> tyreTrackBackRight=new ArrayList<>(MAX_TRACK_ENTRIES+10);

    public static void clearTrack() {
        track=new ArrayList<>(MAX_TRACK_ENTRIES+10);
        tyreTrackFrontLeft=new ArrayList<>(MAX_TRACK_ENTRIES+10);
        tyreTrackFrontRight=new ArrayList<>(MAX_TRACK_ENTRIES+10);
        tyreTrackBackLeft=new ArrayList<>(MAX_TRACK_ENTRIES+10);
        tyreTrackBackRight=new ArrayList<>(MAX_TRACK_ENTRIES+10);
    }


    private int width=-1;  // Dimensionen des Widgets
    private int height=-1;

    private int centreX=-1;
    private int centreY=-1;

    private double pixelPerCM=0.7d;
    private double gridCM=100.0d;


    private double scale=1.0d;  // Zoom-Faktor
    private int screenPosX=0;       // Delta durch Verschieben per Drag
    private int screenPosY=0;

    private boolean followRobot=false; // Soll der Bildschirm den Robot immer in der Mitte zeigen?

    private int horizonY;   // Horizont innerhalb des Camera-Bildes

    private boolean dragging=false;
    private int dragStartX=0;
    private int dragStartY=0;
    private int screenPosStartX=0;
    private int screenPosStartY=0;

// Pose-Gr��en aus DCMotorController kopiert
    private double currentPosX=Double.NaN;
    private double currentPosY=Double.NaN;
    private double currentAngle=Double.NaN;
    private double currentSina=Double.NaN;
    private double currentCosa=Double.NaN;
    private double cameraX=Double.NaN;
    private double cameraY=Double.NaN;


    private Geometry collisionGeometry=null; // Wenn Robot gemalt wurde: welche Fl�che zu Kollisionserkennung heranziehen
    public int collisionsCount=0;            // Wieviele Kollisionen gab es schon
    public long totalCollisionsMS=0;         // Wie lange dauerte diese Kollisionen
    public boolean lastCollision=false;      // Gab es beim letzten Malen schon eine Kollision
    public long lastCollisionMS=-1l;         // Letztes Auftreten einer Kollision

// F�r CopyRight-Bildschirm
    private long msAlive=0;                  // Damit man die Zeit f�r die Copyright-Nachricht messen kann
    private boolean disableCopyright=false;  // Wenn man irgendwas am Canvas macht, Copyright sofort l�schen
    private ImageIcon carbotLogo=null;
    private final Font copyrightFont=new Font("Serif",Font.BOLD,33);
    private final Font copyrightFontSmall=new Font("SansSerif",Font.BOLD,20);


// Remote Debugging Camera Image
    private BufferedImage remoteDebugCameraImage=null;


    public SimCanvas() {
        addMouseListener(this);
        addMouseMotionListener(this);

        new Thread(new Runnable() { 
                public void run() {
                    try {
                        while (true) {
                             try { 
                                 Thread.sleep(REPAINT_CYCLE);
                                 msAlive+=REPAINT_CYCLE;
                             } catch (Exception e) {}
                             repaint();
                        }
                    }
                    catch (Exception e) {
                        System.out.println("Exception in SimCanvas repaint Thread "+e);
                        e.printStackTrace();
                    }
                }
              }).start();
	}
    
    public void disableCopyright() {
        disableCopyright=true;
    }


    public void zoomIn() {
        disableCopyright();
        scale=scale*ZOOM_INTOUT_SCALE;

        screenPosX=(int)Math.round(screenPosX*ZOOM_INTOUT_SCALE);
        screenPosY=(int)Math.round(screenPosY*ZOOM_INTOUT_SCALE);
        
        repaint();
    }


    public void zoomOut() {
        disableCopyright();
        
        scale=scale/ZOOM_INTOUT_SCALE;
        
        screenPosX=(int)Math.round(screenPosX/ZOOM_INTOUT_SCALE);
        screenPosY=(int)Math.round(screenPosY/ZOOM_INTOUT_SCALE);
        
        repaint();
    }


    public void setFollowRobot(boolean followRobot) {
        this.followRobot=followRobot;
    }


    public void paintComponent(Graphics gOrg){

        currentPosX=DCMotorController.currentPosX;
        currentPosY=DCMotorController.currentPosY;


        currentAngle=DCMotorController.currentAngle;
        currentSina=Math.sin(currentAngle*PIDIV180);
        currentCosa=Math.cos(currentAngle*PIDIV180);

        cameraX=Robot.camDist*currentSina+currentPosX;
        cameraY=Robot.camDist*currentCosa+currentPosY;

        width=getWidth();
        height=getHeight();

        centreX=width/2;
        centreY=height/2;
        
        Graphics2D g=(Graphics2D)gOrg;  
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON); // Antialiasing einschalten



        if (followRobot) {   // Der Bildschirm soll dem Robot folgen
            screenPosX=-(int)Math.round(currentPosX*scale*pixelPerCM);
            screenPosY=(int)Math.round(currentPosY*scale*pixelPerCM);
        }



        CMGeom cmGeom=new CMGeom(g,centreX,centreY,screenPosX,screenPosY,scale,pixelPerCM);

        // Hintergrund wei�
        g.setColor(Color.white);
        g.fillRect(0,0,width-1,height-1); 


        // Grid Malen
        if (showGrid) {
            paintGrid(cmGeom,g);
        }


        // Slicks malen
        if (showSlick) {
            Font slickFont=new Font("Serif",Font.BOLD,(int)Math.round(24*scale));
            FontMetrics slickFontMet=g.getFontMetrics(slickFont);
            g.setFont(slickFont); 
            for (int i=0;i<Environment.slicks.size();i++) {
                Slick slick=Environment.slicks.get(i);
                int gr=255-(int)Math.round(((slick.slickLeft+slick.slickRight)/2-1.0d)*10);
                if (gr<100)
                    gr=100;
                Color col=new Color(gr,gr,gr,200);
                cmGeom.drawPoly(slick.positions,Color.black,col,1.5f);
            
                if (!slick.name.equals(""))
                    cmGeom.drawStringCentredCM(Color.blue,slickFontMet,slick.name,slick.centreX,slick.centreY,false);
            }
        }


        // Schatten malen 
        if (showShaddow && Environment.allshaddowsJTSGeom!=null) { 
            cmGeom.paintJTSGeometry(Environment.allshaddowsJTSGeom,null,SHADDOW_COL,0.1d);
        }

        // Track verwalten und malen
        addTrack(track,currentPosX,currentPosY);
        if (showTrack)
            paintTrack(cmGeom,track,TRACK_COL,4.0d);

        // Reifenspuren malen
        if (showTyreTracks) {
            paintTrack(cmGeom,tyreTrackFrontLeft,TYRETRACK_COL,5.0d); 
            paintTrack(cmGeom,tyreTrackFrontRight,TYRETRACK_COL,5.0d);
            paintTrack(cmGeom,tyreTrackBackLeft,TYRETRACK_COL,5.0d);
            paintTrack(cmGeom,tyreTrackBackRight,TYRETRACK_COL,5.0d);
        }

        // Wand malen (nur Realwalls)
        if (Environment.wallsJTSGeom!=null) { 
            cmGeom.paintJTSGeometry(Environment.wallsJTSGeom,Color.black,WALL_COL,0.1d);
        }

        // Wand malen (nur Obstacles)
        if (Environment.obstaclesJTSGeom!=null) { 
            cmGeom.paintJTSGeometry(Environment.obstaclesJTSGeom,Color.black,OBSTACLE_COL,0.1d);
        }
        
        // Wand malen (nur Boxes)
        if (Environment.boxesJTSGeom!=null) { 
            cmGeom.paintJTSGeometry(Environment.boxesJTSGeom,Color.black,BOX_COL,0.1d);
        }        

        // Wand malen (nur die Flat)
        if (Environment.flatsJTSGeom!=null) { 
            cmGeom.paintJTSGeometry(Environment.flatsJTSGeom,Color.black,FLAT_COL,0.1d);
        }


        // Robot Malen
        if (Tetrix.isUp) {

            paintRobot(cmGeom,g);

            // Gab es eine Ausl�sung durch den Taktil-Sensor?
            if (showCollisions && Environment.lastTaktileHitPoint!=null) {
                g.setColor(Color.blue);
                int[] collisionPixel=cmGeom.cm2pixel(Environment.lastTaktileHitPoint[0],Environment.lastTaktileHitPoint[1]);
                int rad=Math.max(4,(int)Math.round(2*scale));
                g.fillOval(collisionPixel[0]-rad,collisionPixel[1]-rad,rad*2,rad*2);              // Kollisionspunkt mit Taktilsensor
            }

            // Gab es eine nahe Messung durch den US-Sensor?
            if (showUS && Environment.lastUSHit!=null) {

                g.setColor(new Color(100,255,100,150));
                g.setStroke(new BasicStroke((float)(2.0*scale)));

                try {

                    int[] usPixel1=cmGeom.cm2pixel(Environment.lastUSHit[0][0],Environment.lastUSHit[0][1]);
                    int[] usPixel2=cmGeom.cm2pixel(Environment.lastUSHit[1][0],Environment.lastUSHit[1][1]);

                    g.drawLine(usPixel1[0],usPixel1[1],usPixel2[0],usPixel2[1]);
                }
                catch (NullPointerException e) {           // Unsch�n: in seltenen F�llen ist lastUSHit beim Malen wieder null - das m�sste ich eigentlich �ber synchronized l�sen :-(
                    System.out.println("US measurement dissapeared during repaint");
                }
            }


            // Gab es eine Kollision mit Hindernissen?
            double[] collisionPoint=Environment.overlappingPointWithWall(collisionGeometry);
            if (collisionPoint!=null) {  // Es gab eine Kollision
                if (showCollisions) {
                    g.setColor(Color.red);
                    int[] collisionPixel=cmGeom.cm2pixel(collisionPoint[0],collisionPoint[1]);
                    int rad=Math.max(4,(int)Math.round(2*scale));
                    g.fillOval(collisionPixel[0]-rad,collisionPixel[1]-rad,rad*2,rad*2);              // Kollisionspunkt mit Robotk�rper malen
                }

                long currentTimeMS=System.currentTimeMillis();
                if (lastCollision) {
                    totalCollisionsMS+=currentTimeMS-lastCollisionMS;
                }
                else {
                    Sound.startRumbleSound();
                    lastCollision=true;
                    collisionsCount++;
                }
                lastCollisionMS=currentTimeMS;
            }
            else { // Es gab keine Kollision
                if (lastCollision)
                    Sound.stopRumbleSound();

                lastCollision=false;
            }

            // Wenn gew�nscht den Positions/Richtungspfeil der internen Position
            if (showInternalPose)
                paintInternalPose(cmGeom,g);

            // Den Sichtkegel braucht man f�r die KeyPoints und die 3D-Ansicht
            Geometry viewAreaKeyPoints=viewArea(MIN_VIEW_DIST,MAX_VIEW_DIST_KEYPOINTS); 
            Geometry viewArea=viewArea(MIN_VIEW_DIST,MAX_VIEW_DIST);

            // Die WallSlices braucht man f�r die KeyPoints und die 3D-Ansicht
            ArrayList<Slice> wallSlices=getWallSlices(cmGeom,viewArea);
            Collections.sort(wallSlices);  // Nach Entfernung sortieren und sp�ter von Vorne durchgehen, damit die Fl�chen, die am ehesten verdecken als erstes gepr�ft werden


            // Alle KeyPoints beschaffen
            ArrayList<InternalKeyPoint3D> keyPointsInView3D=CarbotSim.keyPointRecognizer.computeKeyPoints3D(viewAreaKeyPoints,
                                                                                                    currentPosX,currentPosY,currentAngle,
                                                                                                    cameraX,cameraY,currentSina,currentCosa,
                                                                                                    wallSlices);  // Dem "Recognizer" die letzten Keypoints mitteilen
//            CarbotSim.keyPointRecognizer3D.enterKeyPoints3D(keyPointsInView3D);

            // Keypoints auf Karte von oben malen
            if (showKeypoints) {
                for (InternalKeyPoint3D keypoint:keyPointsInView3D) {
                    g.setColor(keypoint.observed ? KEYPOINT_OBSERVED_COL : KEYPOINT_COL);
                    
                    int[] keypointPixel=cmGeom.cm2pixel(keypoint.x,keypoint.y);
                    int widthHalf=(int)Math.round(3*scale);
                    g.fillRect(keypointPixel[0]- widthHalf,keypointPixel[1]-widthHalf,widthHalf*2,widthHalf*2);
                }
            }

            ((SimDebugPainter)Robot.debugPainter).paint(g,cmGeom);


            if (remoteDebugCameraImage!=null) {  // Remote-Debugging und Camera-Bild wird �bertragen
                paintRealCameraImage(g,remoteDebugCameraImage);
            }
            else if (camLook!=3)  // Wenn gew�nscht ein 3D-Kamerabild malen
                paintCameraImage(cmGeom,g,viewArea,keyPointsInView3D,wallSlices);
            	


            if (msAlive<COPYRIGHT_TIME && !disableCopyright) {
                paintCopyright(g);
            }
        }
    }
    
    public void setBounds(Bounds bounds) {
		super.setBounds(bounds.getX(), bounds.getY(), bounds.getWidth(), bounds.getHeight());
	}

    private int[] camWinRect() {


        int camSizeX=-1;
        int camSizeY=-1;

        switch (camSize) {
           case 0:  // 0: Org
               camSizeX=CAMERA_IMAGE_SIZEX;
               camSizeY=CAMERA_IMAGE_SIZEY;
               break;

           case 1:  // 1: 2x
               camSizeX=CAMERA_IMAGE_SIZEX+CAMERA_IMAGE_SIZEX/2;
               camSizeY=CAMERA_IMAGE_SIZEY+CAMERA_IMAGE_SIZEY/2;
               break;

           case 2:  // 2: 3x, 
               camSizeX=CAMERA_IMAGE_SIZEX*2;
               camSizeY=CAMERA_IMAGE_SIZEY*2;
               break;

           case 3:  // 3: 4x
               camSizeX=CAMERA_IMAGE_SIZEX*2+CAMERA_IMAGE_SIZEX/2;
               camSizeY=CAMERA_IMAGE_SIZEY*2+CAMERA_IMAGE_SIZEY/2;
               break;

           case 4:  // 4: 5x
               camSizeX=CAMERA_IMAGE_SIZEX*3;
               camSizeY=CAMERA_IMAGE_SIZEY*3;
               break;

           case 5:  // 5: max
               camSizeX=width;
               camSizeY=height;
               break; 

           default:
                System.out.println("Illegal CamSize!");
        }
        if (camSizeX>width)
            camSizeX=width;
        if (camSizeY>height)
            camSizeY=height;

        if (camSizeX>camSizeY*CAMERA_IMAGE_SIZEX/CAMERA_IMAGE_SIZEY)
            camSizeX=camSizeY*CAMERA_IMAGE_SIZEX/CAMERA_IMAGE_SIZEY;
        else if (camSizeY>camSizeX*CAMERA_IMAGE_SIZEY/CAMERA_IMAGE_SIZEX)
            camSizeY=camSizeX*CAMERA_IMAGE_SIZEY/CAMERA_IMAGE_SIZEX;


        int camFromX=-1;
        int camFromY=-1;

        switch (camLoc) {
           case 0:   // 0: Upper left
                camFromX=CAMERA_OFFSET_X;
                camFromY=CAMERA_OFFSET_Y;
                break;

           case 1:   // 1: Upper right
                camFromX=width-CAMERA_OFFSET_X-camSizeX;
                camFromY=CAMERA_OFFSET_Y;
                break;

           case 2:   // 2: Lower left 
                camFromX=CAMERA_OFFSET_X;
                camFromY=height-CAMERA_OFFSET_Y-camSizeY;
                break;

           case 3:   // 3: Lower right
                camFromX=width-CAMERA_OFFSET_X-camSizeX;
                camFromY=height-CAMERA_OFFSET_Y-camSizeY;
                break;

           case 4:   // 4: Centre
                camFromX=(width-camSizeX)/2;
                camFromY=(height-camSizeY)/2;
                break;

           default:
                System.out.println("Illegal CamLoc!");
        }


        if (camSizeX+CAMERA_OFFSET_X*2>width) {
            camFromX=(width-camSizeX)/2;
        }
        if (camSizeY+CAMERA_OFFSET_Y*2>height) {
            camFromY=(height-camSizeY)/2;
        }



        return new int[]{camFromX,camFromY,camSizeX,camSizeY};
    }


    private void paintRealCameraImage(Graphics2D g,BufferedImage remoteDebugCameraImage) {

        int[] camWinRect=camWinRect();
        Camera camera=new Camera(g,camWinRect[0],camWinRect[1],camWinRect[2],camWinRect[3]);

        // Camera-Bild malen
        camera.drawImage(remoteDebugCameraImage);

        // An Ende Rahmen malen
        camera.drawFrame();
    }


    private void paintCameraImage(CMGeom cmGeom,Graphics2D g,Geometry viewArea,ArrayList<InternalKeyPoint3D> keyPointsInView3D,ArrayList<Slice> wallSlices) {

        int[] camWinRect=camWinRect();
        Projection projection=new Projection(camWinRect[2],camWinRect[3],cameraX,cameraY,currentSina,currentCosa);

        // Sichtkegel malen
        if (showCamRange) 
            cmGeom.paintJTSGeometry(viewArea,null,new Color(255,255,100,100),0.2d);

        Camera camera=new Camera(g,camWinRect[0],camWinRect[1],camWinRect[2],camWinRect[3]);

        Geometry viewAreaTracks=viewArea(MIN_VIEW_DIST,MAX_VIEW_DIST_TRACKS);
        
        // Fl�che leermachen (zweigeteilt mit Horizon)
        horizonY=(int)Math.round(-Robot.sinCamAlpha/Robot.cosCamAlpha*camWinRect[3]/2/Robot.tanCamVerViewAngleHalf+camWinRect[3]/2);
        camera.drawHorizon(horizonY);


        // Track malen
        if (showTrack && camLook!=2) {
            paintTrackCamera(camera,projection,cmGeom,track,viewAreaTracks,TRACK_COL,6.0d);
        }

        // Reifenspuren malen
        if (showTyreTracks && camLook!=2) {
            paintTrackCamera(camera,projection,cmGeom,tyreTrackFrontLeft,viewAreaTracks,TYRETRACK_COL,6.0d); 
            paintTrackCamera(camera,projection,cmGeom,tyreTrackFrontRight,viewAreaTracks,TYRETRACK_COL,6.0d);
            paintTrackCamera(camera,projection,cmGeom,tyreTrackBackLeft,viewAreaTracks,TYRETRACK_COL,6.0d);
            paintTrackCamera(camera,projection,cmGeom,tyreTrackBackRight,viewAreaTracks,TYRETRACK_COL,6.0d);
        }

        // 3D Schatten Malen
        if (showShaddow && camLook!=2 && Environment.allshaddowsJTSGeom!=null) { 
            try {
                Geometry viewShaddow=Environment.allshaddowsJTSGeom.intersection(viewArea);

                if (!viewShaddow.isEmpty() && (viewShaddow instanceof Polygon || viewShaddow instanceof MultiPolygon)) {
                    ArrayList<ArrayList<double[][]>> polygones3D=cmGeom.cmFromJTSPolyGeometry(viewShaddow);              // Z-Koordinate (=0) ist schon hinzugef�gt

                    for (int i=0;i<polygones3D.size();i++) {
                        ArrayList<int[][]> projection2d=projection.project2D(polygones3D.get(i));
                        camera.drawPolycameraImage(projection2d,null,SHADDOW_COL,0.1d);
                    }
                }
            }
            catch (TopologyException e) { // Kann leider vorkommen
//                System.out.println("Cannot intersect (check1): "+e);   // Passiert sehr h�ufig - nicht mehr melden
            }; 
        }


        // Wand malen (nur die Flats)
        if (Environment.flatsJTSGeom!=null) { 
            try {
                Geometry viewFlats=Environment.flatsJTSGeom.intersection(viewArea);

                if (!viewFlats.isEmpty() && (viewFlats instanceof Polygon || viewFlats instanceof MultiPolygon)) {
                    ArrayList<ArrayList<double[][]>> polygones3D=cmGeom.cmFromJTSPolyGeometry(viewFlats);              // Z-Koordinate (=0) ist schon hinzugef�gt

                    for (int i=0;i<polygones3D.size();i++) {
                        ArrayList<int[][]> projection2d=projection.project2D(polygones3D.get(i));
                        if (camLook==2) {  // Wireframe
                            camera.drawPolycameraImage(projection2d,Color.black,null,1.0d);
                        }
                        else {
                            camera.drawPolycameraImage(projection2d,null,FLAT_COL,0.1d);
                        }
                    }
                }
            }
            catch (TopologyException e) { // Kann leider vorkommen
                System.out.println("Cannot intersect (check2): "+e);
            }; 
        }

        // Jetzt alle anderen W�nde malen, das nach dem "Maler-Algorithmus"
        // Hierf�r wurden die WallSlices schon �bergeben
        if (camLook==2) {  // Wireframe
            for (int i=0;i<wallSlices.size();i++) {
                Slice slice=wallSlices.get(i);
                int[][] projection2d=projection.project2D(slice.getPoly3D());

                Color col=null;
                if (slice.getType()==Wall.REAL_WALL) {
                    col=WALL_COL;
                }
                else if (slice.getType()==Wall.OBSTACLE) {
                    col=OBSTACLE_COL;
                }
                else  {
                    col=BOX_COL;
                }
                ArrayList<int[][]> projectionPoly2D=new ArrayList<>();
                projectionPoly2D.add(projection2d);  // es gibt nur "Shell"
                camera.drawPolycameraImage(projectionPoly2D,col,null,1.0d);
            }
        }
        else {  // Solid oder Glass
//            Collections.sort(wallSlices);   // Schon oben sortiert
            for (int i=0;i<wallSlices.size();i++) {
                Slice slice=wallSlices.get(i);
                int[][] projection2d=projection.project2D(slice.getPoly3D());
                int red;
                int green;
                int blue;
                if (slice.getType()==Wall.REAL_WALL) {
                    red=WALL_COL.getRed();
                    green=WALL_COL.getGreen();
                    blue=WALL_COL.getBlue();
                 }
                else if (slice.getType()==Wall.OBSTACLE) {
                    red=OBSTACLE_COL.getRed();
                    green=OBSTACLE_COL.getGreen();
                    blue=OBSTACLE_COL.getBlue();
                }
                else {
                    red=BOX_COL.getRed();
                    green=BOX_COL.getGreen();
                    blue=BOX_COL.getBlue();
                }

                double angleFact=slice.getCosNorthAngle()*0.5d+1.0d;
                red=Math.min(255,(int)Math.round(red*angleFact));
                green=Math.min(255,(int)Math.round(green*angleFact));
                blue=Math.min(255,(int)Math.round(blue*angleFact));

                int alpha=255;
                if (camLook==1)  // Glass oder 
                    alpha=100;
              
                Color col=new Color(red,green,blue,alpha);

                ArrayList<int[][]> projectionPoly2D=new ArrayList<>();
                projectionPoly2D.add(projection2d);  // es gibt nur "Shell"
                camera.drawPolycameraImage(projectionPoly2D,col,col,1.5d);
            }
        }


        // KeyPoints Malen
        if (showKeypoints) {
            int widthHalf=3;
            for (int i=0;i<keyPointsInView3D.size();i++) {
                InternalKeyPoint3D kp=keyPointsInView3D.get(i);
                g.setColor(kp.observed ? KEYPOINT_OBSERVED_COL : KEYPOINT_COL);
                  
                int pixXscaled=(int)Math.round(1.0d*kp.pixelX*camWinRect[2]/Robot.cameraSizeX);
                int pixYscaled=(int)Math.round(1.0d*kp.pixelY*camWinRect[3]/Robot.cameraSizeY);

                int[] pixel=camera.toCameraPixel(pixXscaled,pixYscaled);

// ALT:               int[] projection2d=projection.project2D(keyPointsInView3D.get(i).getPosition());
//                int[] pixel=camera.toCameraPixel(projection2d[0],projection2d[1]);

                g.fillRect(pixel[0]-widthHalf,pixel[1]-widthHalf,widthHalf*2,widthHalf*2);
            }
        } 

        // Clipregion l�schen
        g.setClip(null);

        // An Ende Rahmen malen
        camera.drawFrame();
    }


    public ArrayList<Slice> getWallSlices(CMGeom cmGeom,Geometry viewArea) {
        ArrayList<Slice> wallSlices=new ArrayList<>(200);   // Alle Wand-"Scheiben"
        
        // real walls
        if (Environment.wallsJTSGeom!=null) {
            try {
                Geometry viewWalls=Environment.wallsJTSGeom.intersection(viewArea);
                if (!viewWalls.isEmpty()) {
                    addWallSlices(cmGeom,wallSlices,viewWalls,Wall.REAL_WALL);
                }
            }
            catch (TopologyException e) { // Kann leider vorkommen
                System.out.println("Cannot intersect (check3): "+e);
            }; 
        }
        
        // obstacles
        if (Environment.obstaclesJTSGeom!=null) {
            try {
                Geometry viewObstacles=Environment.obstaclesJTSGeom.intersection(viewArea);
                if (!viewObstacles.isEmpty())
                    addWallSlices(cmGeom,wallSlices,viewObstacles,Wall.OBSTACLE);
            }
            catch (TopologyException e) { // Kann leider vorkommen
                System.out.println("Cannot intersect (check4): "+e);
            }; 
        }
        
        // boxes
        if (Environment.boxesJTSGeom!=null) {
            try {
                Geometry viewBoxes=Environment.boxesJTSGeom.intersection(viewArea);
                if (!viewBoxes.isEmpty())
                    addWallSlices(cmGeom,wallSlices,viewBoxes,Wall.BOX);
            }
            catch (TopologyException e) { // Kann leider vorkommen
                System.out.println("Cannot intersect (check4b): "+e);
            }; 
        }        
        
        return wallSlices;
    }


    private void addWallSlices(CMGeom cmGeom,ArrayList<Slice> wallSlices,Geometry geometry,int type) {
        double height=Wall.HEIGHTS[type];
        ArrayList<ArrayList<double[][]>> polygones3D=cmGeom.cmFromJTSPolyGeometry(geometry);
        for (int i=0;i<polygones3D.size();i++) {   // Alle Polys durchgehen 
            ArrayList<double[][]> polygone3D=polygones3D.get(i);
            for (int j=0;j<polygone3D.size();j++) {   // Shell + Holes durchgen
                double[][] points=polygone3D.get(j);
                for (int k=0;k<points.length-1;k++) {
                     double[] p1=points[k];
                     double[] p2=points[k+1];
                     Slice slice=new Slice(type,p1,p2,height,cameraX,cameraY);
                     wallSlices.add(slice);
                }
            }
        }
    }


    private Geometry viewArea(double fromViewDist,double toViewDist) {

        double camera_width_near=fromViewDist*Robot.tanCamHorViewAngleHalf;
        double camera_width_far=toViewDist*Robot.tanCamHorViewAngleHalf;

        Coordinate[] coords=new Coordinate[5];
        coords[0]=new Coordinate(cameraX+currentSina*fromViewDist+currentCosa*camera_width_near,cameraY+currentCosa*fromViewDist-currentSina*camera_width_near);
        coords[1]=new Coordinate(cameraX+currentSina*fromViewDist-currentCosa*camera_width_near,cameraY+currentCosa*fromViewDist+currentSina*camera_width_near);
        coords[2]=new Coordinate(cameraX+currentSina*toViewDist-currentCosa*camera_width_far,cameraY+currentCosa*toViewDist+currentSina*camera_width_far);
        coords[3]=new Coordinate(cameraX+currentSina*toViewDist+currentCosa*camera_width_far,cameraY+currentCosa*toViewDist-currentSina*camera_width_far);
        coords[4]=coords[0];

        return geomfact.createPolygon(geomfact.createLinearRing(coords),new LinearRing[0]);
    }


    private static void addTrack(ArrayList<double[]> theTrack,double posX,double posY) {
        if (theTrack.size()>=MAX_TRACK_ENTRIES)
            theTrack.remove(0);
        if (theTrack.size()==0) {
            theTrack.add(new double[]{posX,posY});
        }
        else {
            double[] lastTrack=theTrack.get(theTrack.size()-1);
            double sqrdist=(lastTrack[0]-posX)*(lastTrack[0]-posX)+(lastTrack[1]-posY)*(lastTrack[1]-posY);
            if (sqrdist>TRACK_MINSQRDIST)
                theTrack.add(new double[]{posX,posY});
        }
    }


    private void paintTrackCamera(Camera camera,Projection projection,CMGeom cmGeom,ArrayList<double[]> theTrack,Geometry viewAreaTracks,Color col,double thickness) {
        if (theTrack.size()<=1)
            return;

        LineString trackGeom=Geom.line2JTSGeometry(theTrack);
        try {
            Geometry viewTrack=trackGeom.intersection(viewAreaTracks);
            if (!viewTrack.isEmpty()) {
                ArrayList<double[][]> track3D=cmGeom.cmFromJTSLineGeometry(viewTrack);              // Z-Koordinate (=0) ist schon hinzugef�gt
                ArrayList<int[][]> projection2d=projection.project2D(track3D);
                camera.drawLinescameraImage(projection2d,col,thickness); 
            }
        }
        catch (TopologyException e) { // Kann leider vorkommen
//            System.out.println("Cannot intersect (check5): "+e);   // Passiert leider h�ufig - Ausgabe nicht machen
        };
    }


    private void paintTrack(CMGeom cmGeom,ArrayList<double[]> theTrack,Color col,double thickness) {
        if (theTrack.size()>1) {
            LineString line=Geom.line2JTSGeometry(theTrack);
            cmGeom.paintJTSGeometry(line,col,null,thickness);
        }
    }


    private void paintInternalPose(CMGeom cmGeom,Graphics2D g) {
    
        if (Double.isNaN(CarbotSim.asyncValueX) ||
            Double.isNaN(CarbotSim.asyncValueY) ||
            Double.isNaN(CarbotSim.asyncValueANGLE)) {
             return;
        }

        int[] posPixel=cmGeom.cm2pixel(CarbotSim.asyncValueX,CarbotSim.asyncValueY);
          
        int[] posPixel2=cmGeom.cm2pixel(CarbotSim.asyncValueX+CarbotSim.sinAsyncValueANGLE*20,
                                        CarbotSim.asyncValueY+CarbotSim.cosAsyncValueANGLE*20);

        int rad=Math.max(6,(int)Math.round(3*scale));
        g.setColor(Color.green);
        g.setStroke(new BasicStroke((float)(3.0f*scale)));
        g.fillOval(posPixel[0]-rad,posPixel[1]-rad,rad*2,rad*2);              // Kollisionspunkt mit Robotk�rper malen
        g.drawLine(posPixel[0],posPixel[1],posPixel2[0],posPixel2[1]);

        g.setStroke(new BasicStroke((float)(1.0f*scale)));
        int[] posPixelArrow1=cmGeom.cm2pixel(CarbotSim.asyncValueX+CarbotSim.sinAsyncValueANGLE*40,
                                             CarbotSim.asyncValueY+CarbotSim.cosAsyncValueANGLE*40);
        int[] posPixelArrow2=cmGeom.cm2pixel(CarbotSim.asyncValueX+CarbotSim.sinAsyncValueANGLE*20-CarbotSim.cosAsyncValueANGLE*10,
                                             CarbotSim.asyncValueY+CarbotSim.cosAsyncValueANGLE*20+CarbotSim.sinAsyncValueANGLE*10);
        int[] posPixelArrow3=cmGeom.cm2pixel(CarbotSim.asyncValueX+CarbotSim.sinAsyncValueANGLE*20+CarbotSim.cosAsyncValueANGLE*10,
                                             CarbotSim.asyncValueY+CarbotSim.cosAsyncValueANGLE*20-CarbotSim.sinAsyncValueANGLE*10);

        int[] x=new int[3];
        int[] y=new int[3];
        x[0]=posPixelArrow1[0];   y[0]=posPixelArrow1[1];
        x[1]=posPixelArrow2[0];   y[1]=posPixelArrow2[1];
        x[2]=posPixelArrow3[0];   y[2]=posPixelArrow3[1];
 
        g.fillPolygon(x,y,x.length);
    }


    private void paintGrid(CMGeom cmGeom,Graphics2D g) {
       
        g.setColor(Color.blue);
        g.setFont(gridFont); 

        double cmX=0;
        while (cmGeom.cm2pixel(cmX,0)[0]<width) {
            drawGridLineX(cmGeom,g,cmX);
            cmX+=gridCM;
        }

        cmX=0;
        while (cmGeom.cm2pixel(cmX,0)[0]>=0) {
            drawGridLineX(cmGeom,g,cmX);
            cmX-=gridCM;
        }

        double cmY=0;
        while (cmGeom.cm2pixel(0,cmY)[1]>=0) {
            drawGridLineY(cmGeom,g,cmY);
            cmY+=gridCM;
        }

        cmY=0;
        while (cmGeom.cm2pixel(0,cmY)[1]<height) {
            drawGridLineY(cmGeom,g,cmY);
            cmY-=gridCM;
        }
    }


    private void drawGridLineX(CMGeom cmGeom,Graphics2D g,double cmX) {
         int[] pix=cmGeom.cm2pixel(cmX,0);
         int m=(int)Math.round(cmX/100);
         boolean major=(m % 5==0);

         g.setStroke(major ? strokeNormal : strokeThin);
         g.drawLine(pix[0],0,pix[0],height-1);  

         if (major) {
             String text=""+m+"m";
             g.drawString(text,pix[0]+4,height-5);
         }
    }


    private void drawGridLineY(CMGeom cmGeom,Graphics2D g,double cmY) {
         int[] pix=cmGeom.cm2pixel(0,cmY);
         int m=(int)Math.round(cmY/100);
         boolean major=(m % 5==0);

         g.setStroke(major ? strokeNormal : strokeThin);
         g.drawLine(0,pix[1],width-1,pix[1]);  

         if (major) {
             String text=""+m+"m";
             g.drawString(text,0,pix[1]-2);
         }
    }


    private void paintRobot(CMGeom cmGeom,Graphics2D g) {

        float servoAngleLeft=0.0f;
        float servoAngleRight=0.0f;

        if (Tetrix.servoLeft!=null) {
            servoAngleLeft=Tetrix.servoLeft.getAngle()-Tetrix.leftSteerOffset;
            servoAngleRight=Tetrix.servoRight.getAngle()-Tetrix.rightSteerOffset;
        }

        int[] pos=cmGeom.cm2pixel(currentPosX,currentPosY);
        g.setColor(Color.black);
        g.fillOval(pos[0]-5,pos[1]-5,10,10);


        Coordinate[] collisionShape=new Coordinate[17];  // Rahmen zur Kollisionserkennung: wird durch drawBody und draw...Wheel belegt

        drawFrontWheel(cmGeom,-Tetrix.trackWidth/2,0,currentAngle,currentPosX,currentPosY,collisionShape);
        drawFrontWheel(cmGeom, Tetrix.trackWidth/2,0,currentAngle,currentPosX,currentPosY,collisionShape);

        drawBackWheel(cmGeom,-Tetrix.steerBase/2,-Tetrix.wheelBase,-servoAngleLeft, -(Tetrix.trackWidth-Tetrix.steerBase)/2,currentAngle,currentPosX,currentPosY,collisionShape);
        drawBackWheel(cmGeom, Tetrix.steerBase/2,-Tetrix.wheelBase,-servoAngleRight, (Tetrix.trackWidth-Tetrix.steerBase)/2,currentAngle,currentPosX,currentPosY,collisionShape);

        drawBody(cmGeom,currentAngle,currentPosX,currentPosY,collisionShape);
        collisionShape[16]=collisionShape[0];

        collisionGeometry=geomfact.createPolygon(geomfact.createLinearRing(collisionShape),new LinearRing[0]);
    }


    private void drawBody(CMGeom cmGeom,double robotAngle,double robotX,double robotY,Coordinate[] collisionShape) {

        // K�rperumriss
        double[][] body=new double[][] {
             {-Tetrix.trackWidth/2, BODY_LENGTH_FRONT-6.0d},    // 0: Vorderkante links hinten
             {-Tetrix.trackWidth/2, BODY_LENGTH_FRONT},         // 1: Vorderkante linke vorne
             { Tetrix.trackWidth/2, BODY_LENGTH_FRONT},         // 2: Vorderkante rechts vorne
             { Tetrix.trackWidth/2, BODY_LENGTH_FRONT-6.0d},    // 3: Vorderkante rechts hinten
             { BODY_WIDTH/2, BODY_LENGTH_FRONT-6.0d},           // 4:
             { BODY_WIDTH/2,-10.0d},                            // 5: Punkt wird nur f�r Collisionsshape ben�tigt
             { BODY_WIDTH/2,-BODY_LENGTH_BACK},                 // 6: Hinterkante rechts
             {-BODY_WIDTH/2,-BODY_LENGTH_BACK},                 // 7: Hinterkante links
             {-BODY_WIDTH/2,-10.0d},                            // 8: Punkt wird nur f�r Collisionsshape ben�tigt
             {-BODY_WIDTH/2, BODY_LENGTH_FRONT-6.0d},           // 9:

        };
        Geom.rotateAll(body,-robotAngle);
        Geom.moveAll(body,robotX,robotY);
        cmGeom.drawPoly(body,Color.black,new Color(200,200,200),0.3f);


        collisionShape[0]=new Coordinate(body[0][0],body[0][1]);  // 0: Vorderkante links hinten
        collisionShape[1]=new Coordinate(body[1][0],body[1][1]);  // 1: Vorderkante linke vorne
        collisionShape[2]=new Coordinate(body[2][0],body[2][1]);  // 2: Vorderkante rechts vorne
        collisionShape[3]=new Coordinate(body[3][0],body[3][1]);  // 3: Vorderkante rechts hinten
        // 4-5: Rechtes Vorderrad
        collisionShape[6]=new Coordinate(body[5][0],body[5][1]);  // 5: Punkt wird nur f�r Collisionsshape ben�tigt
        // 7-8: Rechtes Hinterrad
        collisionShape[9]=new Coordinate(body[6][0],body[6][1]);  // 6: Hinterkante rechts
        collisionShape[10]=new Coordinate(body[7][0],body[7][1]); // 7: Hinterkante links
        // 11-12: Linkes Hinterrad
        collisionShape[13]=new Coordinate(body[8][0],body[8][1]); // 8: Punkt wird nur f�r Collisionsshape ben�tigt
        // 14-15: Linkes Vorderrad


        // Raspi
        double[][] bodyComp1=new double[][] {
             {-BODY_WIDTH/5,BODY_LENGTH_FRONT-5.0d},    
             { BODY_WIDTH/5,BODY_LENGTH_FRONT-5.0d},
             { BODY_WIDTH/5,BODY_LENGTH_FRONT-15.0d},
             {-BODY_WIDTH/5,BODY_LENGTH_FRONT-15.0d}
        };
        Geom.rotateAll(bodyComp1,-robotAngle);
        Geom.moveAll(bodyComp1,robotX,robotY);
        cmGeom.drawPoly(bodyComp1,Color.black,new Color(150,150,255),0.1f);

        // NXT-Baustein
        double[][] bodyComp2=new double[][] {
             {-BODY_WIDTH/5,-BODY_LENGTH_BACK+5.0d},    
             { BODY_WIDTH/5,-BODY_LENGTH_BACK+5.0d},
             { BODY_WIDTH/5,-BODY_LENGTH_BACK+15.0d},
             {-BODY_WIDTH/5,-BODY_LENGTH_BACK+15.0d}
        };
        Geom.rotateAll(bodyComp2,-robotAngle);
        Geom.moveAll(bodyComp2,robotX,robotY);
        cmGeom.drawPoly(bodyComp2,Color.black,Color.white,0.1f);

        // Taktil-Sensor
        double[][] bodyTaktile=new double[][] {
             {-Robot.taktileWidth/2, Robot.taktileDist},    
             { Robot.taktileWidth/2, Robot.taktileDist},
             { Robot.taktileWidth/2, Robot.taktileDist-1.0d},
             {-Robot.taktileWidth/2, Robot.taktileDist-1.0d}
        };
        Geom.rotateAll(bodyTaktile,-robotAngle);
        Geom.moveAll(bodyTaktile,robotX,robotY);
        cmGeom.drawPoly(bodyTaktile,Color.black,Color.black,0.1f);
    }



    private void drawFrontWheel(CMGeom cmGeom,double inRobotX,double inRobotY,double robotAngle,double robotX,double robotY,Coordinate[] collisionShape) {

        double sgn=inRobotX>0 ? -1.0d : 1.0d;

        double[][] axis=new double[][] {
             {-WHEEL_WIDTH*sgn,  -1.0d},
             { 10.0d*sgn,        -1.0d},
             { 10.0d*sgn,         1.0d},
             {-WHEEL_WIDTH*sgn,   1.0d}
        };

        Geom.moveAll(axis,inRobotX,inRobotY);
        Geom.rotateAll(axis,-robotAngle);
        Geom.moveAll(axis,robotX,robotY);
        cmGeom.drawPoly(axis,Color.gray,new Color(200,100,0),0.1f);


        double[][] wheel=new double[][] {
             {-WHEEL_WIDTH/2,-Tetrix.wheelDiameter},    // /2 fehlt, da der angegeben wheelDiameter genau die H�lft des echten Raddurchmessers ist, wegen der Untersetzung durch die Zahnr�der
             { WHEEL_WIDTH/2,-Tetrix.wheelDiameter},
             { WHEEL_WIDTH/2, Tetrix.wheelDiameter},
             {-WHEEL_WIDTH/2, Tetrix.wheelDiameter}
        };
        Geom.moveAll(wheel,inRobotX,inRobotY);
        Geom.rotateAll(wheel,-robotAngle);
        Geom.moveAll(wheel,robotX,robotY);
        cmGeom.drawPoly(wheel,Color.gray,Color.black,0.1f);


        if (inRobotX>0) {  // Rechtes Rad
            collisionShape[4]=new Coordinate(wheel[2][0],wheel[2][1]);     // Rechtes Vorderrad vorne
            collisionShape[5]=new Coordinate(wheel[1][0],wheel[1][1]);     // Rechtes Vorderrad hinten
            addTrack(tyreTrackFrontRight,(wheel[0][0]+wheel[2][0])/2,(wheel[0][1]+wheel[2][1])/2);
        }
        else {
            collisionShape[14]=new Coordinate(wheel[0][0],wheel[0][1]);    // Linkes Vorderrad hinten
            collisionShape[15]=new Coordinate(wheel[3][0],wheel[3][1]);    // Linkes Vorderrad vorne
            addTrack(tyreTrackFrontLeft,(wheel[0][0]+wheel[2][0])/2,(wheel[0][1]+wheel[2][1])/2);
        }

    }


    private void drawBackWheel(CMGeom cmGeom,double anchorX,double anchorY,double steerAngle,double steerDist,double robotAngle,double robotX,double robotY,Coordinate[] collisionShape) {

        double sgn=anchorX>0 ? -1.0d : 1.0d;

        // Erst Achse
        double[][] axis=new double[][] {
             {-WHEEL_WIDTH*sgn,  -1.0d},
             { 10.0d*sgn,        -1.0d},
             { 10.0d*sgn,         1.0d},
             {-WHEEL_WIDTH*sgn,   1.0d}
        };

        Geom.moveAll(axis,steerDist,0);
        Geom.rotateAll(axis,steerAngle);
        Geom.moveAll(axis,anchorX,anchorY);
        Geom.rotateAll(axis,-robotAngle);
        Geom.moveAll(axis,robotX,robotY);
        cmGeom.drawPoly(axis,Color.gray,new Color(200,100,0),0.1f);

        // Dann das eigentliche Rad

        double[][] wheel=new double[][] {
             {-WHEEL_WIDTH/2,-Tetrix.wheelDiameter},    // /2 fehlt, da der angegeben wheelDiameter genau die H�lfte des echten Raddurchmessers ist, wegen der Untersetzung durch die Zahnr�der
             { WHEEL_WIDTH/2,-Tetrix.wheelDiameter},
             { WHEEL_WIDTH/2, Tetrix.wheelDiameter},
             {-WHEEL_WIDTH/2, Tetrix.wheelDiameter}
        };
        Geom.moveAll(wheel,steerDist,0);
        Geom.rotateAll(wheel,steerAngle);
        Geom.moveAll(wheel,anchorX,anchorY);
        Geom.rotateAll(wheel,-robotAngle);
        Geom.moveAll(wheel,robotX,robotY);
        cmGeom.drawPoly(wheel,Color.gray,Color.black,0.1f);


        if (anchorX>0) {  // Rechtes Rad
            collisionShape[7]=new Coordinate(wheel[2][0],wheel[2][1]);    // Rechtes Hinterrad vorne
            collisionShape[8]=new Coordinate(wheel[1][0],wheel[1][1]);    // Rechtes Hinterrad hinten
            addTrack(tyreTrackBackRight,(wheel[0][0]+wheel[2][0])/2,(wheel[0][1]+wheel[2][1])/2);
        }
        else {            // Linkes Rad
            collisionShape[11]=new Coordinate(wheel[0][0],wheel[0][1]);    // Linkes Hinterrad hinten
            collisionShape[12]=new Coordinate(wheel[3][0],wheel[3][1]);    // Linkes Hinterrad vorne
            addTrack(tyreTrackBackLeft,(wheel[0][0]+wheel[2][0])/2,(wheel[0][1]+wheel[2][1])/2);
        }
    }




    public void mousePressed(MouseEvent e) {
        disableCopyright();
        dragging=true;
        dragStartX=e.getX();
        dragStartY=e.getY();
        screenPosStartX=screenPosX;
        screenPosStartY=screenPosY;
    }


    public void mouseClicked(MouseEvent e) {
    }


    public void mouseReleased(MouseEvent e) {
        dragging=false;
    }


    public void mouseDragged(MouseEvent e) {
        if (!dragging) 
            return;

        int deltaDragX=e.getX()-dragStartX;
        int deltaDragY=e.getY()-dragStartY;

        screenPosX=screenPosStartX+deltaDragX;
        screenPosY=screenPosStartY+deltaDragY;

        repaint();
    }


    public void mouseMoved(MouseEvent e) {
    }


    public void mouseEntered(MouseEvent e) {
    }


    public void mouseExited(MouseEvent e) {
    }


// Malen des kleinen Copyright-Vermerks
    private void paintCopyright(Graphics2D g) {


        int alpha=255-(int)(1000-COPYRIGHT_TIME+msAlive)/4;
        if (alpha>255)
            alpha=255;

        int centreX=width/2;
        int centreY=height/2;

        int boxWidth=300;
        int boxHeight=280;

        g.setColor(new Color(255,255,189,alpha));
        g.fillRect(centreX-boxWidth/2,centreY-boxHeight/2,boxWidth,boxHeight);

        g.setColor(new Color(0,0,100,alpha));
        g.setStroke(new BasicStroke(2.0f));

        g.drawRect(centreX-boxWidth/2,centreY-boxHeight/2,boxWidth,boxHeight);

        if (carbotLogo==null) {
            try {
                carbotLogo=new ImageIcon(CarbotSim.class.getClassLoader().getResource("resources/carbotlogo.png"));
            }
            catch (Exception e) {
            }
        }

//        carbotLogo.paintIcon(this, g, centreX-80,centreY-boxHeight/2+20);


        Composite currentComposite=g.getComposite();
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, alpha/255.0f));
        g.drawImage(carbotLogo.getImage(),centreX-80,centreY-boxHeight/2+20,null);
        g.setComposite(currentComposite);

        g.setFont(copyrightFont);
        g.drawString("Carbot Simulator",centreX-120,centreY-10);
        g.drawString("by J\u00f6rg Roth",centreX-120,centreY+25);

        g.setFont(copyrightFontSmall);
        g.drawString("Version",   centreX-120,centreY+75);
        g.drawString(VERSION_STR, centreX+70,centreY+75);

        g.drawString("Motion Subsystem",centreX-120,centreY+100);
        g.drawString(Tetrix.VERSION,   centreX+70,centreY+100);

        g.drawString("Robot Interface",centreX-120,centreY+125);
        g.drawString(Robot.VERSION,    centreX+70,centreY+125);

    }



    public void setRemoteDebugCameraImage(BufferedImage image) {
        remoteDebugCameraImage=image;
    }



}