package sim.keypoints;

// Simulierter Key-Point
// Achtung: diese Struktur "kennt" Fakten, die ein echter Keypoint nicht kennen würde
// So gibt es eine internalID, die ihn eindeutig in der Umgebung macht
// Echte Keypoints würde nur wissen, dass sie in einer Bildfolge immer derselbe Keypoint ist.
// Ich brauche diese internalID, um die 2D-Keypoints einander zuzuordnen


/**
* An internal simulated keypoint in 3D. Caution: this structure is only internal for the simulator and must not be passed
* to code that should run on a real robot. 
* The reason: this structur 'knows' fact that a real keypoint cannot know. E.g. there is an internalID that related this keypoint
* uniquely to the environment. In reality, this is not possible.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class InternalKeyPoint3D {

    private static int freeID=1;

/** ID that relates the keypoint uniquely to the environment (e.g. to a certain object corner). */
    public int internalID;

/** The ID that is replied as <i>lineID</i> for 2D keypoints. */
    public int lineID;

/** 3D position x */
    public double x;  

/** 3D position y */
    public double y;  

/** 3D position z (height) */
    public double z;  

/** Pixel position (x) if viewed by the camera (0: centre: +/- Robot.cameraSizeX/2, negative: left) */
    public int pixelX=Integer.MIN_VALUE;  

/** Pixel position (y) if viewed by the camera (0: centre: +/- Robot.cameraSizeY/2, negative: top) */
    public int pixelY=Integer.MIN_VALUE;  

/** Is this keypoint observed, i.e. on two subsequent camera images are taken from different positions. */
    public boolean observed=false;


/** 
* Creates a new keypoint, assigns a unique ID
* @param x 3D position x
* @param y 3D position y
* @param z 3D position z (height)
*/
    public InternalKeyPoint3D(double x,double y,double z) {
        this.internalID=freeID++;
        this.lineID=internalID;
        this.x=x;
        this.y=y;
        this.z=z;
    }


/**
* Returns the postion.
* @return array of 3 with coordinates.
*/
    public double[] getPosition() {
        return new double[]{x,y,z};
    }

}