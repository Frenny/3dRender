package sim.util;

import java.awt.Color;

/**
* Internal class to modell single commands in the context of debug painting.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class DebugPaintCommand {

    public int type; // 1: fillSquare, 2: fillSquare, 3: cross, 4: drawText, 5: line, 6: ellipse

    public double cmX;
    public double cmY;

    public double cmX2;
    public double cmY2;

    public double cmSize;
    public double cmSizeHalf;


    public double cmRad1;
    public double cmRad2;
    public double rotAlpha;


    public String text;
    public boolean centered;
    public boolean clear;

    public Color color;


    public DebugPaintCommand(int type,double cmX, double cmY,double cmX2, double cmY2, double cmSize, 
                             double cmRad1,double cmRad2,double rotAlpha,
                             String text, boolean centered,boolean clear,int red, int green, int blue, int alpha) {
        this.type=type;

        this.cmX=cmX;
        this.cmY=cmY;

        this.cmX2=cmX2;
        this.cmY2=cmY2;

        this.cmSize=cmSize;
        this.cmSizeHalf=cmSize/2;

        this.cmRad1=cmRad1;
        this.cmRad2=cmRad2;
        this.rotAlpha=rotAlpha;

        this.text=text;
        this.centered=centered;
        this.clear=clear;

        color=new Color(red,green,blue,alpha);
    }
}