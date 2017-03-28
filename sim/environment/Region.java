package sim.environment;

import java.util.ArrayList;


import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Coordinate;

/**
* A region is anything that resides in the virtual environment.
*/
public abstract class Region {

     private static GeometryFactory geomfact=new GeometryFactory();

     public String name="";

     public double height=Double.NaN;

     public double[][] positions=null;  // Polygon des Bereichs
     public double centreX=Double.NaN;
     public double centreY=Double.NaN;

     public Geometry jtsGeom=null;          // Die Region des Objektes
     public Geometry jtsShadowGeom=null;    // Das Schattenpolygon

     private double maxX=Double.NaN;
     private double maxY=Double.NaN;
     private double minX=Double.NaN;
     private double minY=Double.NaN;


/**
* Creates a region. As Region is abstract, the Region cannot be instantiated directly.
* @param name a name
* @param positions the ground positions (cm, only x,y)
* @param height the object's height
* @param hasShaddow does the object has a shaddow
*/
     public Region(String name,ArrayList<double[]> positions,double height,boolean hasShaddow) {
         this.name=name;
         this.height=height;
         this.positions=new double[positions.size()][];

         maxX=-1e10;
         maxY=-1e10;
         minX=1e10;
         minY=1e10;

         Coordinate[] coords=new Coordinate[positions.size()+1];

         for (int i=0;i<positions.size();i++) {
             this.positions[i]=positions.get(i);
             coords[i]=new Coordinate(this.positions[i][0],this.positions[i][1]);
             maxX=Math.max(maxX,this.positions[i][0]);
             maxY=Math.max(maxY,this.positions[i][1]);
             minX=Math.min(minX,this.positions[i][0]);
             minY=Math.min(minY,this.positions[i][1]);
         }
         coords[coords.length-1]=coords[0];

         jtsGeom=geomfact.createPolygon(geomfact.createLinearRing(coords),new LinearRing[0]);

         centreX=(maxX+minX)/2;
         centreY=(maxY+minY)/2;


         // Das SchattenPolygon berechnen

         double shaddowOffsetX=height/4;
         double shaddowOffsetY=-height/6;

         if (hasShaddow) {
             jtsShadowGeom=jtsGeom;  // Mit der Grundfläche initalisieren
             for (int i=1;i<coords.length;i++) {
                 Coordinate[] shaddowParalax=new Coordinate[5];
                 shaddowParalax[0]=coords[i-1];
                 shaddowParalax[1]=coords[i];
                 shaddowParalax[2]=new Coordinate(coords[i].x+shaddowOffsetX,coords[i].y+shaddowOffsetY);
                 shaddowParalax[3]=new Coordinate(coords[i-1].x+shaddowOffsetX,coords[i-1].y+shaddowOffsetY);
                 shaddowParalax[4]=shaddowParalax[0];

                 Geometry paralaxJtsGeom=geomfact.createPolygon(geomfact.createLinearRing(shaddowParalax),new LinearRing[0]);

                 jtsShadowGeom=jtsShadowGeom.union(paralaxJtsGeom);
             }
         }
     }


/**
* Does the region cover this position (ground projection, cm).
* @param cmX the position (cm)
* @param cmY the position (cm)
* @return covers?
*/
     public boolean contains(double cmX,double cmY) {
         if (cmX>maxX || cmX<minX || cmY>maxY || cmY<minY)   // Box-Text zur Beschleunigung
             return false;

         Point point=geomfact.createPoint(new Coordinate(cmX,cmY));
         return jtsGeom.contains(point);
     }

}