package sim.geom;

import java.util.ArrayList;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.BasicStroke;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;


/**
* The Camera allows us to paint the 3D image of the simulated camera view.
* An instance is only used to paint, not for any 3D computations (e.g. for key points).
* This object only accepts pixel position, where as (0,0) is the image centre and increasing y goes down.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Camera {

    private final static BasicStroke strokeCamFrame = new BasicStroke(5);
    private final static Color CAM_IMAGE_FLOOR_COL=new Color(245,245,245);
    private final static Color CAM_IMAGE_SKY_COL=new Color(220,230,255);


    private int camScreenX;
    private int camScreenY;
    private int camScreenSizeX;
    private int camScreenSizeY;
    private Graphics2D g;

/**
* Creates an object to paint the camera view.
* @param g the graphics 2d of the painting window
* @param camScreenX upper left corner of the camera image on the painting window
* @param camScreenY upper left corner of the camera image on the painting window
* @param camScreenSizeX size of the camera image on the painting window
* @param camScreenSizeY size of the camera image on the painting window
*/
    public Camera(Graphics2D g,int camScreenX,int camScreenY,int camScreenSizeX,int camScreenSizeY) {
        this.g=g;
        this.camScreenX=camScreenX;
        this.camScreenY=camScreenY;
        this.camScreenSizeX=camScreenSizeX;
        this.camScreenSizeY=camScreenSizeY;

        g.setClip(camScreenX,camScreenY,camScreenSizeX,camScreenSizeY); 
    }


/**
* Draw two areas (bottom, sky) on the screen, indicating the horizon.
* @param horizonY horizon line (0: in the centre)
*/
    public void drawHorizon(int horizonY) {
        if (horizonY<0) {  // Nur "Boden"
           g.setColor(CAM_IMAGE_FLOOR_COL);
           g.fillRect(camScreenX,camScreenY,camScreenSizeX,camScreenSizeY); 

        }
        else if (horizonY>=camScreenSizeY) {  // Nur Himmel
           g.setColor(CAM_IMAGE_SKY_COL);
           g.fillRect(camScreenX,camScreenY,camScreenSizeX,camScreenSizeY); 
        }
        else {
           g.setColor(CAM_IMAGE_FLOOR_COL);
           g.fillRect(camScreenX,camScreenY+horizonY,camScreenSizeX,camScreenSizeY-horizonY); 
           g.setColor(CAM_IMAGE_SKY_COL);
           g.fillRect(camScreenX,camScreenY,camScreenSizeX,horizonY); 
        }
    }


/**
* Draws an image (usually the real camera via remote debugging) on the camera screen. 
* @param image the image
*/
    public void drawImage(BufferedImage image) {
        g.drawImage(image,camScreenX,camScreenY,camScreenSizeX,camScreenSizeY,null);
    }


/**
* Draws rectangle around the image on the screen.
*/
    public void drawFrame() {
        g.setColor(Color.black);
        g.setStroke(strokeCamFrame);
        g.drawLine(camScreenX,               camScreenY,camScreenX+camScreenSizeX,camScreenY);
        g.drawLine(camScreenX+camScreenSizeX,camScreenY,camScreenX+camScreenSizeX,camScreenY+camScreenSizeY);
        g.drawLine(camScreenX+camScreenSizeX,camScreenY+camScreenSizeY,camScreenX,camScreenY+camScreenSizeY);
        g.drawLine(camScreenX,camScreenY+camScreenSizeY,camScreenX,camScreenY);
    }


/**
* Transforms a camera pixel (0,0): centre to a screen pixel (respecting the area on the screen reserved for the camera image).
* @param pixelX camera pixel position x (0: centre)
* @param pixelY camera pixel position y (0: centre)
* @return screen pixel position
*/
    public int[] toCameraPixel(int pixelX,int pixelY) {
        return new int[] {
            pixelX+camScreenX+camScreenSizeX/2,
            pixelY+camScreenY+camScreenSizeY/2
          };
    }


// Hilsfunktion: ein Poly (Shell oder Hole) in Array umkopieren, so das man direkt daraus ein awt.Polygon machen kann.
    private int[][] polyArray(int[][] poly2D) {
        int numPoints=poly2D.length;
        if (poly2D[0][0]==poly2D[numPoints-1][0] && poly2D[0][1]==poly2D[numPoints-1][1])
            numPoints--;

        int[] x=new int[numPoints];
        int[] y=new int[numPoints];

        for (int i=0;i<numPoints;i++) {
            x[i]=poly2D[i][0]+camScreenX+camScreenSizeX/2;
            y[i]=poly2D[i][1]+camScreenY+camScreenSizeY/2;
        }
        return new int[][] {x,y};
    }


/**
* Draw a single polygon with holes. The list contains the polygons shell as the first element and the rest of the list contains holes.
* @param poly2D single polygon with multiple holes
* @param borderColor border color
* @param fillColor fill color
* @param thickness border thickness in pixels
*/
    public void drawPolycameraImage(ArrayList<int[][]> poly2D,Color borderColor,Color fillColor,double thickness) {
        int[][] polyArray=polyArray(poly2D.get(0));   // Shell
        Area polyArea=new Area(new java.awt.Polygon(polyArray[0], polyArray[1], polyArray[0].length));

        for (int i=1;i<poly2D.size();i++) {
            polyArray=polyArray(poly2D.get(i));   // Hole
            polyArea.subtract(new Area(new java.awt.Polygon(polyArray[0], polyArray[1], polyArray[0].length)));
        }

        if (fillColor!=null) {
            g.setColor(fillColor);
            g.fill(polyArea);
        }

        if (borderColor!=null) {
            g.setColor(borderColor);
            g.setStroke(new BasicStroke((float)(thickness)));
            g.draw(polyArea);
        }
    }


/**
* Draw multiple lines on the camera image.
* @param lines2D multiple lines, each list element is a linestring of pixel positions
* @param lineColor line color
* @param thickness line thickness in pixels
*/
    public void drawLinescameraImage(ArrayList<int[][]> lines2D,Color lineColor,double thickness) {
        g.setColor(lineColor);
        g.setStroke(new BasicStroke((float)(thickness)));

        for (int i=0;i<lines2D.size();i++) {
            int[][] line=lines2D.get(i);

            Path2D linePath=new Path2D.Double();
            linePath.moveTo(line[0][0]+camScreenX+camScreenSizeX/2,
                            line[0][1]+camScreenY+camScreenSizeY/2
                           );

            for (int j=1;j<line.length;j++) {
                linePath.lineTo(line[j][0]+camScreenX+camScreenSizeX/2,
                                line[j][1]+camScreenY+camScreenSizeY/2
                               );
            }
            g.draw(linePath);
        }
    }

}
