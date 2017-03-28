package sim.geom;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.Coordinate;

import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.BasicStroke;
import java.awt.FontMetrics;
import java.awt.geom.Area;
import java.awt.geom.Path2D;

import robotinterface.obj.LRUCache;


/**
* Support of conversion of cm (simluated world) in pixels (screen) and vice versa.
* Some methods draw directly on the screen using cm measures.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class CMGeom {

    private Graphics2D g;


    private int centreX=-1;
    private int centreY=-1;

    private int screenPosX;       // Delta durch Verschieben per Drag
    private int screenPosY;

    public double scale;         // Zoom-Faktor
    private double pixelPerCM;

    private double pixelPerCMtimesScale;



    private final static int CACHE_ENTRIES_POLY=20;  // Eigentlich werden nur 2-3 Polys gecacht - mehr, damit keine später mal mehr gemalt werden kann
    private final static int CACHE_ENTRIES_LINE=20;  // Eigentlich werden nur 3-4 Lines gecacht - mehr, damit keine später mal mehr gemalt werden kann


    // Alles für Caches von Poly->AWTArea
    private static int cachedcentreX=Integer.MIN_VALUE;
    private static int cachedcentreY=Integer.MIN_VALUE;
    private static int cachedscreenPosX=Integer.MIN_VALUE;
    private static int cachedscreenPosY=Integer.MIN_VALUE;
    private static double cachedpixelPerCMtimesScale=0.0d;
    private static LRUCache<Polygon,Area> poly2areaCache=new LRUCache<>(CACHE_ENTRIES_POLY);
    private static LRUCache<Geometry,Path2D> line2pathCache=new LRUCache<>(CACHE_ENTRIES_LINE);



/**
* Instantiates a conversion between cm and pixels.
* @param g graphics 2d object to paint on the screen
* @param centreX pixel centre of the window
* @param centreY pixel centre of the window
* @param screenPosX pixel offset as a result of dragging the window
* @param screenPosY pixel offset as a result of dragging the window
* @param scale scale for zooming: 1.0: normal view &gt;1.0 zooming in, &lt;1.0 zooming out
* @param pixelPerCM basic relation of pixel and cm (for zooming scale 1.0)
*/
    public CMGeom(Graphics2D g,int centreX,int centreY,int screenPosX,int screenPosY,double scale,double pixelPerCM) {

        this.g=g;
        this.centreX=centreX;
        this.centreY=centreY;
        this.screenPosX=screenPosX;
        this.screenPosY=screenPosY;
        this.scale=scale;
        this.pixelPerCM=pixelPerCM;

        pixelPerCMtimesScale=pixelPerCM*scale;

        if (centreX!=cachedcentreX || centreY!=cachedcentreY ||
            screenPosX!=cachedscreenPosX || screenPosY!=cachedscreenPosY ||
            pixelPerCMtimesScale!=cachedpixelPerCMtimesScale) {
            poly2areaCache=new LRUCache<>(CACHE_ENTRIES_POLY);
            line2pathCache=new LRUCache<>(CACHE_ENTRIES_LINE);

            cachedcentreX=centreX;
            cachedcentreY=centreY;
            cachedscreenPosX=screenPosX;
            cachedscreenPosY=screenPosY;
            cachedpixelPerCMtimesScale=pixelPerCMtimesScale;
        }

    }


/**
* Convert cm to screen pixels.
* @param cmX world position
* @param cmY world position
* @return array of 2: [0]: pixel position x, [1]: pixel position y
*/
    public int[] cm2pixel(double cmX,double cmY) {
        return new int[] {
            (int)Math.round(centreX+cmX*pixelPerCMtimesScale)+screenPosX,
            (int)Math.round(centreY-cmY*pixelPerCMtimesScale)+screenPosY
        };
    }


/**
* Convert screen pixels to cm. 
* @param pixX pixel position
* @param pixY pixel position
* @return array of 2: [0]: world position x, [1]: world position y
*/
    public double[] pixel2cm(int pixX,int pixY) {
        return new double[] {
            1.0d*(pixX-centreX-screenPosX)/pixelPerCMtimesScale,
            1.0d*(centreY-pixY-screenPosY)/pixelPerCMtimesScale,
        };
    }



/**
* Paint a JTS geometry (in cm) directly on the screen.
* @param geom a JTS geometry, that may be Point, LineString, Polygon or GeometryCollection of these
* @param borderColor polygon border or line string color
* @param fillColor polygon fill color
* @param thickness border or line string thickness in pixels (of scale 1.0)
*/
    public void paintJTSGeometry(Geometry geom,Color borderColor,Color fillColor,double thickness) {
        if (geom instanceof Point) {
            Coordinate coord=((Point)geom).getCoordinate();
            int[] xy=cm2pixel(coord.x,coord.y);
            g.setColor(borderColor);
            int rad=(int)Math.round(scale*thickness);
            g.fillOval(xy[0]-rad/2,xy[1]-rad/2,rad,rad);
        }
        else if (geom instanceof LineString) {
            Path2D linePath=null;
            linePath=line2pathCache.get(geom);
            if (linePath==null) {
                Coordinate[] lineCoords=geom.getCoordinates();
                linePath=new Path2D.Double();
                for (int i=0;i<lineCoords.length;i++) {
                    Coordinate coord=lineCoords[i];
                    int[] xy=cm2pixel(coord.x,coord.y);
                    if (i==0)
                        linePath.moveTo(xy[0],xy[1]);
                    else
                    linePath.lineTo(xy[0],xy[1]);
                }
                line2pathCache.put(geom,linePath);
            }
            g.setColor(borderColor);
            g.setStroke(new BasicStroke((float)(thickness*scale)));
            g.draw(linePath);
        }
        else if (geom instanceof Polygon) {
            Area polyArea;
            Polygon poly=(Polygon)geom;

            polyArea=poly2areaCache.get(poly);
            if (polyArea==null) {
                LineString extring=poly.getExteriorRing();
                int numpoints=extring.getNumPoints();

                int[] xx = new int[numpoints];
                int[] yy = new int[numpoints];

                for (int i=0;i<numpoints;i++) {
                    Coordinate coord=extring.getCoordinateN(i);
                    int[] xy=cm2pixel(coord.x,coord.y);

                    xx[i]=xy[0];
                    yy[i]=xy[1];
                }

                polyArea=new Area(new java.awt.Polygon(xx, yy, xx.length));
 
                for (int i=0;i<poly.getNumInteriorRing();i++)  {  // Löscher "subtrahieren"
                    LineString intring=poly.getInteriorRingN(i);
                    int numpointsInt=intring.getNumPoints();
                    int[] xxInt = new int[numpointsInt];
                    int[] yyInt = new int[numpointsInt];

                    for (int j=0;j<numpointsInt;j++) {
                        Coordinate coordInt=intring.getCoordinateN(j);
                        int[] xy=cm2pixel(coordInt.x,coordInt.y);
                        xxInt[j]=xy[0];
                        yyInt[j]=xy[1];
                    }  
                    polyArea.subtract(new Area(new java.awt.Polygon(xxInt,yyInt,xxInt.length)));
                }
                poly2areaCache.put(poly,polyArea);
            }

            if (fillColor!=null) {
                g.setColor(fillColor);
                g.fill(polyArea);
            }

            if (borderColor!=null) {
                g.setColor(borderColor);
                g.setStroke(new BasicStroke((float)(thickness*scale)));
                g.draw(polyArea);
            }
            
        }
        else if (geom instanceof GeometryCollection) {
            for (int i=0;i<geom.getNumGeometries();i++) {
                paintJTSGeometry(geom.getGeometryN(i),borderColor,fillColor,thickness);
            }
        }
        else {
            throw new IllegalArgumentException("don't know how to paint buffer paintJTSGeometry geometry "+geom.getClass().getName());
        }
    }


/**
* Paint a polyon (in cm, without holes) directly on the screen.
* @param polyCM the polygon, consists of an array of positions
* @param border polygon border color
* @param fill polygon fill color
* @param thickness border thickness in pixels (of scale 1.0)
*/
    public void drawPoly(double[][] polyCM,Color border,Color fill,double thickness) {
        int[] x=new int[polyCM.length];
        int[] y=new int[polyCM.length];

        for (int i=0;i<polyCM.length;i++) {
            int[] pixPos=cm2pixel(polyCM[i][0],polyCM[i][1]);
            x[i]=pixPos[0];
            y[i]=pixPos[1];
        }

        g.setColor(fill);
        g.fillPolygon(x,y,x.length);

        g.setColor(border);
        g.setStroke(new BasicStroke((float)(thickness*scale)));
        g.drawPolygon(x,y,x.length);
    }


/**
* Computes world coordinates (in cm) of a given JTS line string geometry.
* For easy further processing, the world coordinates are three dimensional, wheras z is always 0.0d.
* @param geom a JTS geometry, that may be LineString or GeometryCollection of LineString
* @return list of line strings, each contains an array of points, each {x, y, 0}
*/
    public ArrayList<double[][]> cmFromJTSLineGeometry(Geometry geom) {
        if (geom instanceof LineString) {
            ArrayList<double[][]> resultLines=new ArrayList<>(); 

            LineString lineString=(LineString)geom;

            int numpoints=lineString.getNumPoints();
            double[][] points=new double[numpoints][];

            for (int i=0;i<numpoints;i++) {
                Coordinate coord=lineString.getCoordinateN(i);
                points[i]=new double[]{coord.x,coord.y,0.0d};
            }
            resultLines.add(points);
            return resultLines;
        }
        else if (geom instanceof GeometryCollection) {
            ArrayList<double[][]> result=new ArrayList<>(30);
            for (int i=0;i<geom.getNumGeometries();i++) {
                ArrayList<double[][]> resI=cmFromJTSLineGeometry(geom.getGeometryN(i));
                result.addAll(resI);
            }
            return result;
        }
        else {
            throw new IllegalArgumentException("don't know how to cmFromJTSLineGeometry from geometry "+geom.getClass().getName());
        }
    }


/**
* Computes world coordinates (in cm) of a given JTS polygon geometry.
* For easy further processing, the world coordinates are three dimensional, wheras z is always 0.0d.
* @param geom a JTS geometry, that may be Polygon or GeometryCollection of Polygon
* @return list of polygons, each contains a list, where as the first element is a shell, further list elements holes, each of these contains an array of points, each {x, y, 0}
*/
    public static ArrayList<ArrayList<double[][]>> cmFromJTSPolyGeometry(Geometry geom) {
        if (geom instanceof Polygon) {

            ArrayList<ArrayList<double[][]>> result=new ArrayList<>();

            Polygon poly=(Polygon)geom;

            ArrayList<double[][]> resultPoly=new ArrayList<>(); 

            LineString extring=poly.getExteriorRing();
            int numpoints=extring.getNumPoints();
            double[][] points=new double[numpoints][];

            for (int i=0;i<numpoints;i++) {
                Coordinate coord=extring.getCoordinateN(i);
                points[i]=new double[]{coord.x,coord.y,0.0d};
            }
            resultPoly.add(points);

            for (int i=0;i<poly.getNumInteriorRing();i++)  {  // Löscher 
                LineString intring=poly.getInteriorRingN(i);

                numpoints=intring.getNumPoints();
                points=new double[numpoints][];

                for (int j=0;j<numpoints;j++) {
                    Coordinate coord=intring.getCoordinateN(j);
                    points[j]=new double[]{coord.x,coord.y,0.0d};
                }
                resultPoly.add(points);
            }
            result.add(resultPoly);
            return result;
        }
        else if (geom instanceof GeometryCollection) {
            ArrayList<ArrayList<double[][]>> result=new ArrayList<>(30);
            for (int i=0;i<geom.getNumGeometries();i++) {
                ArrayList<ArrayList<double[][]>> resI=cmFromJTSPolyGeometry(geom.getGeometryN(i));
                result.addAll(resI);
            }
            return result;
        }
        else {
            throw new IllegalArgumentException("don't know how to cmFromJTSPolyGeometry from geometry "+geom.getClass().getName());
        }
    }


/**
* Paint a text on the screen, whereas the centre of the text is given in cm.
* @param color text color
* @param fontMet the text font metrics (caution, setFont already has to be performed)
* @param text the one-line text to paint
* @param cmX world position of the text
* @param cmY world position of the text
* @param clear clear the text background before painting
*/
    public void drawStringCentredCM(Color color,FontMetrics fontMet,String text,double cmX,double cmY,boolean clear) {
        int[] pos=cm2pixel(cmX,cmY);

        int width=fontMet.stringWidth(text);
        int height=fontMet.getHeight();

        if (clear) {
            g.setColor(Color.white);
            g.fillRect(pos[0]-width/2,pos[1]-height/2,width,height);
        }
        g.setColor(color);
        g.drawString(text,pos[0]-width/2,pos[1]+height/5);
    }


/**
* Paint a text on the screen, whereas the left side of the text is given in cm.
* @param color text color
* @param fontMet the text font metrics (caution, setFont already has to be performed)
* @param text the one-line text to paint
* @param cmX world position of the text
* @param cmY world position of the text
* @param clear clear the text background before painting
*/
    public void drawStringLeftCM(Color color,FontMetrics fontMet,String text,double cmX,double cmY,boolean clear) {
        int[] pos=cm2pixel(cmX,cmY);

        int width=fontMet.stringWidth(text);
        int height=fontMet.getHeight();

        if (clear) {
            g.setColor(Color.white);
            g.fillRect(pos[0],pos[1]-height/2,width,height);
        }

        g.setColor(color);
        g.drawString(text,pos[0],pos[1]+height/5);  
    }




}