package sim.geom;

import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Point;


// Die Auﬂenfl‰che einer Wand oder eines Hindernisses
// Diese Fl‰che muss senkrecht stehen
/**
* A slice is a surface of a wall or obstacle. A slice is always vertical (i.e. the projection on the bottom is a line).
* Its lower border is on the bottom and it has a certain height.
* A slice is used to compute the 3D camera image, in particular the paint ordering to hide hidden objects.
* In addition slices are used to remove not visibale key points.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Slice implements Comparable<Slice> {

    private static GeometryFactory geomfact=new GeometryFactory();

    private int type; 

    private double p1x;
    private double p1y;
    private double p2x;
    private double p2y;
    private double height;
    private double cosNorthAngle;

    private double[][] poly3D=null;
    private double sqrDistToCam1;
    private double sqrDistToCam2;



/**
* Creates a slice.
* @param type the type according Wall
* @param p1 the first position (cm), projected on the bottom, no height
* @param p2 the first position (cm), projected on the bottom, no height
* @param height the height of the slice
* @param camX the camera position (cm), projected on the bottom
* @param camY the camera position (cm), projected on the bottom
*/
    public Slice(int type,double[] p1,double[] p2,double height,double camX,double camY) {
        this.type=type;
        p1x=p1[0];
        p1y=p1[1];
        p2x=p2[0];
        p2y=p2[1];
        this.height=height;

        double dx=p2x-p1x;
        double dy=p2y-p1y;
        cosNorthAngle=dy/Math.sqrt(dx*dx+dy*dy);  // Cosinussatz cos alpha=a*b/(|a|*|b|)  (northVektor ist (0,1)

        poly3D=new double[5][];
        poly3D[0]=new double[]{p1x,p1y,0.0d};   // Zuerst Boden
        poly3D[1]=new double[]{p2x,p2y,0.0d};
        poly3D[2]=new double[]{p2x,p2y,height}; // Dann Oberkante
        poly3D[3]=new double[]{p1x,p1y,height};
        poly3D[4]=poly3D[0];

        sqrDistToCam1=(p1x-camX)*(p1x-camX)+(p1y-camY)*(p1y-camY);
        sqrDistToCam2=(p2x-camX)*(p2x-camX)+(p2y-camY)*(p2y-camY);
    }


// Schneidet die 3D-Verbindung zwischen zwei Punkten diese Fl‰che?
/** 
* Does the line given by two positions (3D, cm) intersects with the slice?
* @param x1 first position (3D, cm)
* @param y1 first position (3D, cm)
* @param z1 first position (3D, cm)
* @param x2 second position (3D, cm)
* @param y2 second position (3D, cm)
* @param z2 second position (3D, cm)
* @return true if the line intersects with the slice
*/
    public boolean intersects(double x1,double y1,double z1,double x2,double y2,double z2) {

        double diva=((x2-x1)*(p2y-p1y)-(p2x-p1x)*(y2-y1));
        double divb=((x2-x1)*(p2y-p1y)-(p2x-p1x)*(y2-y1));
        double ix=Double.NaN;
        double iy=Double.NaN;

        if (Math.abs(diva)<1e-10 || Math.abs(divb)<1e-10) {   // Strecken (fast) parallel
            return false;
        }
        else {
            double a=((p1x-x1)*(p2y-p1y)-(p2x-p1x)*(p1y-y1))/
                     diva;
            double b=((p1x-x1)*(y2-y1)-(x2-x1)*(p1y-y1))/
                     divb;
            if (a>1.0d || a<0.0d || b>1.0d || b<0.0d) {       // Geraden schneiden sich nicht innerhalb der Strakcen
                return false;
            }
            else {
                ix=x1+a*(x2-x1);
                iy=y1+a*(y2-y1);
            }
        }


        double distToI=Math.sqrt((ix-x1)*(ix-x1)+(iy-y1)*(iy-y1)); // Abstand Puntk 1 -> Schnittpunkt
        double distTo2=Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)); // Abstand Punkt 1 -> Punkt 2

        if (distTo2-distToI<1.0d)  // Es gibt zwar einen Schnitt, der liegt aber sehr nahe an dieser Wand, daher kommt der Punkt wahrscheinlich von dieser
            return false;          // Damit man diesen Punkt nicht aussortiert, wird "nicht geschnitten" zur¸ckgegeben


        double iz;
        
        if (distTo2<0.1) {   // Rubustheit: wenn die Punkte zu nahe beinander ist die Division problematisch
           iz=(z1+z2)/2;
        }

        else
            iz=z1+(z2-z1)*distToI/distTo2;

        return iz>=0 && iz<=height;  // Die Hˆhe passiert die Fl‰che
    }



/**
* Comparing two slices. &lt;0 if 'this' is farer than the given slice.
* As a result, Collections.sort(List of Slices) orders from 'far' to 'near', what is required to paint all slices accordingly to their distance.
* @param other other slice
* @return result of comparing
*/
    public int compareTo(Slice other) {
        return -Double.compare(this.sqrDistToCam1+this.sqrDistToCam2,other.sqrDistToCam1+other.sqrDistToCam2);
    }


/**
* Type number according to Wall.
* @return type
*/
    public int getType() {
        return type;
    }


/**
* cos(north angle), used to compute the brightness when painting this slice.
* @return cos(angle)
*/
    public double getCosNorthAngle() {
        return cosNorthAngle;
    }

/**
* Returns the 3D polygon (cm) of the entire slice.
* @return list of positions, each x,y,z
*/
    public double[][] getPoly3D() {
        return poly3D;
    }

}