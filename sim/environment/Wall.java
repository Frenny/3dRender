package sim.environment;

import java.util.ArrayList;


/**
* A Wall is as a real wall, an obstacle or an object on the bottom (&quot;flat&quot;).
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Wall extends Region {

/** Heights of the respective wall types. */
     public final static double[] HEIGHTS={
         100.0d,    // REAL_WALL
           2.0d,    // FLAT
          50.0d,    // OBSTACLE
          25.0d     // BOX
     };

/** Type, see constants */
     public int type=-1;  

/** Visible by the Vision sub system (i.e. it can detect keypoints)? */
     public boolean hasKeypoints; 

/** Wall type &quot;real wall&quot; */
     public final static int REAL_WALL=0;

/** Wall type &quot;flat&quot; */
     public final static int FLAT=1;

/** Wall type &quot;obstacle&quot; */
     public final static int OBSTACLE=2;

/** Wall type &quot;box&quot; */
     public final static int BOX=3;


/**
* Creates a Wall object.
* @param type the type accordng to the constants
* @param hasKeypoints is this wall is visible by keypoint detection?
* @param positions the positions (ground projection of the obstacle's shell, no holes)
*/
     public Wall(int type,boolean hasKeypoints,ArrayList<double[]> positions) {
         super(null,positions,HEIGHTS[type],type!=FLAT);
         this.type=type;
         this.hasKeypoints=hasKeypoints;
     }


}