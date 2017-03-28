package sim.geom;

import java.util.ArrayList;

import robotinterface.Robot;

/**
* Computes projections of world coordinates (3D, cm) to pixels of a simulated camera.
* A camera is specified by it resolution (pixels) and its position/orientation in world coordinates (cm, degree).
* In addition to the camera position that is a result of moving among the plane, the camera position is a result of its 'internal' position on the robot.
* In addition, also the mounting angle of the camera (e.g. looking to bottom, looking to sky) is considered. 
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Projection {

    private int camScreenSizeX;
    private int camScreenSizeY;
    private double cameraX;
    private double cameraY;
    private double currentSina;
    private double currentCosa;


/**
* Instantiate a projection.
* @param camScreenSizeX camera resolutin
* @param camScreenSizeY camera resolutin
* @param cameraX world position (in cm) of the camera in the plane
* @param cameraY world position (in cm) of the camera in the plane
* @param currentSina sin(angle) of the current camera orientation
* @param currentCosa sin(angle) of the current camera orientation
*/
    public Projection(int camScreenSizeX,int camScreenSizeY,double cameraX,double cameraY,double currentSina,double currentCosa) {
        this.camScreenSizeX=camScreenSizeX;
        this.camScreenSizeY=camScreenSizeY;
        this.cameraX=cameraX;
        this.cameraY=cameraY;
        this.currentSina=currentSina;
        this.currentCosa=currentCosa;
    }


// 1. Shell. 2, 3, ...n: Holes
/**
* Project a polygon with holes.
* @param polygon polygon with holes (actually, multiple rings)
* @return projected polygon with holes
*/
    public ArrayList<int[][]> project2D(ArrayList<double[][]> polygon) {
        ArrayList<int[][]> result=new ArrayList<>(polygon.size());
        for (int i=0;i<polygon.size();i++) {
             result.add(project2D(polygon.get(i)));
        }
        return result;
    }

/**
* Project a polygon ring (that may be a shell or hole).
* @param polygon polygon ring
* @return projected polygon ring
*/
    public int[][] project2D(double[][] polygon) {
        int[][] result=new int[polygon.length][];
        for (int i=0;i<polygon.length;i++) {
           result[i]=project2D(polygon[i]);
        }
        return result;
    }


/**
* Project a single point.
* @param point point
* @return projected point
*/
    public int[] project2D(double[] point) {
        double x=point[0]-cameraX;
        double y=point[1]-cameraY;
        double z=point[2]-Robot.camHeight;

/* Das hier war falsch
        double rotX=-x*currentCosa+y*currentSina;
        double rotY=-x*currentSina-y*currentCosa;

        double rot2Y=Robot.cosCamAlpha*rotY-Robot.sinCamAlpha*z;
        double rotZ=Robot.sinCamAlpha*rotY-Robot.cosCamAlpha*z;

        double relpixX=rotX/rot2Y;
        double relpixY=-rotZ/rot2Y;
*/

        double rotX= x*currentCosa-y*currentSina;
        double rotY= x*currentSina+y*currentCosa;

        double rot2Y=Robot.cosCamAlpha*rotY-Robot.sinCamAlpha*z;
        double rotZ=+Robot.sinCamAlpha*rotY+Robot.cosCamAlpha*z;


        double relpixX=rotX/rot2Y;
        double relpixY=-rotZ/rot2Y;


        return new int[] {
            (int)Math.round(relpixX/Robot.tanCamHorViewAngleHalf*camScreenSizeX/2),
            (int)Math.round(relpixY/Robot.tanCamVerViewAngleHalf*camScreenSizeY/2)
        };
    }


}