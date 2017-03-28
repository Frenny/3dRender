package sim.geom;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Coordinate;


/**
* Geometric helper functions.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Geom {
    private final static double PIDIV180=Math.PI/180.0d;
    private static GeometryFactory geomfact=new GeometryFactory();


/**
* Rotate position around (0,0).
* @param pos all positions (caution, the result is also stored here)
* @param angle the rotation angle in degrees
*/
    public static void rotateAll(double[][] pos, double angle) {
        double a=angle*PIDIV180;
        double sina=Math.sin(a);
        double cosa=Math.cos(a);

        for (int i=0;i<pos.length;i++) {
            double x=pos[i][0];
            double y=pos[i][1];

            pos[i][0]= cosa*x-sina*y;
            pos[i][1]= sina*x+cosa*y;
        }
    }


/**
* Move positions.
* @param pos all positions (caution, the result is also stored here)
* @param dx movement 
* @param dy movement
*/
    public static void moveAll(double[][] pos, double dx,double dy) {
        for (int i=0;i<pos.length;i++) {
            pos[i][0]+=dx;
            pos[i][1]+=dy;
        }
    }

/**
* Compute a JTS LineString from a list of positions.
* @param line list of positions
* @return the line string
*/
    public static LineString line2JTSGeometry(ArrayList<double[]> line) {
        Coordinate[] coords=new Coordinate[line.size()];
        for (int i=0;i<coords.length;i++) {
            double[] trackPos=line.get(i);
            coords[i]=new Coordinate(trackPos[0],trackPos[1]);
        }
        return geomfact.createLineString(coords);
    }



}