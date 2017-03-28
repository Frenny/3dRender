package sim.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;


import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.BasicStroke;


import sim.geom.CMGeom;


import robotinterface.util.DebugPainter;



/**
* An implementaion of the DebugPainter that paints debug drawings on the simulator's ground map.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class SimDebugPainter implements DebugPainter {

    private final static String DEFAULT_OVERLAY_NAME="Default";

    private ArrayList<String> overlays=null;                                           // Aktuelle Liste der Overlay-Namen
    private HashMap<String,ArrayList<DebugPaintCommand>> overlayCollectCommands=null;  // Overlay-Name -> aktuelle Liste der Commands

    private String currentOverlay=null;                          // Aktuell gesetzer Overlay-Name
    private ArrayList<DebugPaintCommand> collectCommands=null;   // Aktuelle Command-Liste (eine aus overlayCollectCommands)


    private HashSet<String> disabledOverlays=null;               // Liste der Overlays, die nicht angezeigt werden sollen

    private HashMap<String,ArrayList<DebugPaintCommand>> overlayPaintCommands=null;     // Overlay-Name -> Die Commands, die aktuell gemalt werden sollen

    private boolean defaultOverlayChecked;                     // Zum schnellen Test, ob es ein Default-Overlay gibt: false: noch  nicht geprüft, true: geprüft (egal, ob dringelassen oder gelöscht)


    public SimDebugPainter() {
        overlays=new ArrayList<>();
        overlayCollectCommands=new HashMap<>();
        overlayPaintCommands=new HashMap<>();
        setOverlay(DEFAULT_OVERLAY_NAME);
        defaultOverlayChecked=false;
        disabledOverlays=new HashSet<>();
    }


/**
* Return all currently defined overlays.
* @return list of overlay names
*/
    public synchronized ArrayList<String> getOverlays() {
        return overlays;
    }


/**
* Define a set of overlays that should not be painted.
* @param disabledOverlays set of disabled overlays
*/
    public synchronized void setOverlayDisabled(HashSet<String> disabledOverlays) {
        this.disabledOverlays=disabledOverlays;
    }


/**
* Get the set of formerly defined overlays that should not be painted.
* @return set of disabled overlays
*/
    public synchronized HashSet<String> getOverlayDisabled() {
        return disabledOverlays;
    }


/**
* Start painting on an certain overlay. An overlay is a selectable part of all drawings. This implementation of DebugPainter
* support overlays, thus,  each can be switched on and off by the user.
* The overlay is represented by a string, presented to the user. After a setOverlay() is executed, all subsequent
* drawing commands are assigned to this overlay.
* @param overlay overlay name
*/
    public synchronized void setOverlay(String overlay) {
        currentOverlay=overlay;
        collectCommands=overlayCollectCommands.get(overlay); 
        if (collectCommands==null) {
            overlays.add(overlay);
            collectCommands=new ArrayList<>();
            overlayCollectCommands.put(overlay,collectCommands);
            overlayPaintCommands.put(overlay,new ArrayList<>());
        }
    }


/**
* Clear debug output.
*/
    public synchronized void clear() {
        collectCommands.clear();
    }


/**
* Submit the output to screen.
*/
    public synchronized void paint() {

        if (!defaultOverlayChecked) {
            if (!currentOverlay.equals(DEFAULT_OVERLAY_NAME)) {
                if (overlayPaintCommands.get(DEFAULT_OVERLAY_NAME).size()==0) {
                    overlays.remove(0);
                    overlayCollectCommands.remove(DEFAULT_OVERLAY_NAME);
                    overlayPaintCommands.remove(DEFAULT_OVERLAY_NAME);
                }
            }
            defaultOverlayChecked=true;
        }

        ArrayList<DebugPaintCommand> paintCommands=new ArrayList<>();
        paintCommands.addAll(collectCommands); 
        overlayPaintCommands.put(currentOverlay,paintCommands);
    }


/**
* Draw a filled square.
* @param cmX center position x in world coordinates
* @param cmY center position y in world coordinates
* @param cmSize size in cm
* @param red fill color red value
* @param green fill color green value
* @param blue fill color blue value
* @param alpha fill color alpha value
*/
    public synchronized void fillSquare(double cmX, double cmY, double cmSize, int red, int green, int blue, int alpha) {
        collectCommands.add(
              new DebugPaintCommand(1,cmX,cmY,0,0,cmSize,0,0,0,null,false,false,red,green,blue,alpha)
        );
    }


/**
* Draw a filled circle.
* @param cmX center position x in world coordinates
* @param cmY center position y in world coordinates
* @param cmSize size in cm
* @param red fill color red value
* @param green fill color green value
* @param blue fill color blue value
* @param alpha fill color alpha value
*/
    public synchronized void fillCircle(double cmX, double cmY, double cmSize, int red, int green, int blue, int alpha) {
        collectCommands.add(
              new DebugPaintCommand(2,cmX,cmY,0,0,cmSize,0,0,0,null,false,false,red,green,blue,alpha)
        );
    }


/**
* Draw a filled ellipse.
* @param cmX center position x in world coordinates
* @param cmY center position y in world coordinates
* @param cmRad1 radius<sub>1</sub> in cm; radius<sub>1</sub> is the ellipse radius in x-direction (before rotating the ellipse)
* @param cmRad2 radius<sub>2</sub> in cm; radius<sub>2</sub> is the ellipse radius in y-direction (before rotating the ellipse)
* @param rotAlpha rotating angle in degree, goes clockwise; 0 means: radius<sub>1</sub> still is in x-direction, 45: radius<sub>1</sub> is in 45&deg; to lower/right
* @param red fill color red value
* @param green fill color green value
* @param blue fill color blue value
* @param alpha fill color alpha value
*/
    public void fillEllipse(double cmX, double cmY, double cmRad1, double cmRad2,double rotAlpha, int red, int green, int blue, int alpha) {
        collectCommands.add(
              new DebugPaintCommand(6,cmX,cmY,0,0,0,cmRad1,cmRad2,rotAlpha,null,false,false,red,green,blue,alpha)
        );
    }


/**
* Draw a cross ('X').
* @param cmX position x in world coordinates
* @param cmY position y in world coordinates
* @param cmSize size in cm
* @param red draw color red value
* @param green draw color green value
* @param blue draw color blue value
* @param alpha draw color alpha value
*/
    public synchronized void drawCross(double cmX, double cmY, double cmSize, int red, int green, int blue, int alpha) {
        collectCommands.add(
              new DebugPaintCommand(3,cmX,cmY,0,0,cmSize,0,0,0,null,false,false,red,green,blue,alpha)
        );
    }


/**
* Draw a text.
* @param cmX position x in world coordinates
* @param cmY position y in world coordinates
* @param text text (single line)
* @param centered true: the text center position give, false: the left side
* @param clear true: clear the text area before painting the text
* @param cmHeight height of a small letter in cm
* @param red text color red value
* @param green text color green value
* @param blue text color blue value
* @param alpha text color alpha value
*/
    public synchronized void drawText(double cmX, double cmY, String text, boolean centered,boolean clear,double cmHeight,int red, int green, int blue, int alpha) {
        collectCommands.add(
              new DebugPaintCommand(4,cmX,cmY,0,0,cmHeight,0,0,0,text,centered,clear,red,green,blue,alpha)
        );
    }


/**
* Draw a line.
* @param cmX first position x in world coordinates
* @param cmY first position y in world coordinates
* @param cmX2 second position x in world coordinates
* @param cmY2 second position y in world coordinates
* @param red fill color red value
* @param green fill color green value
* @param blue fill color blue value
* @param alpha fill color alpha value
*/
    public synchronized void drawLine(double cmX, double cmY, double cmX2, double cmY2, int red, int green, int blue, int alpha) {
        collectCommands.add(
              new DebugPaintCommand(5,cmX,cmY,cmX2,cmY2,-1,0,0,0,null,false,false,red,green,blue,alpha)
        );
    }


/**
* Draw an arrow.
* @param cmX first position x in world coordinates
* @param cmY first position y in world coordinates
* @param cmX2 second position x in world coordinates
* @param cmY2 second position y in world coordinates
* @param cmArrow length of the two arrow ankles
* @param red fill color red value
* @param green fill color green value
* @param blue fill color blue value
* @param alpha fill color alpha value
*/
    public void drawArrow(double cmX, double cmY, double cmX2, double cmY2, double cmArrow,int red, int green, int blue, int alpha) {
        drawLine(cmX,cmY,cmX2,cmY2,red,green,blue,alpha);
        double dx=cmX2-cmX;
        double dy=cmY2-cmY;
        double len=Math.hypot(dx,dy);
        if (len>5) {
            dx=dx/len*cmArrow;
            dy=dy/len*cmArrow;
            drawLine(cmX2,cmY2,cmX2-dx-dy/2,cmY2-dy+dx/2,red,green,blue,alpha);
            drawLine(cmX2,cmY2,cmX2-dx+dy/2,cmY2-dy-dx/2,red,green,blue,alpha);
        }
    }


/**
* Render the drawing commands on the screen.
* @param g graphics
* @param cmGeom conversion cm to pixel
*/
    public synchronized void paint(Graphics2D g,CMGeom cmGeom) {
        for (String o:overlays) {
            if (!disabledOverlays.contains(o)) {
                ArrayList<DebugPaintCommand> paintCommands=overlayCollectCommands.get(o);
                paint(g,cmGeom,paintCommands);
            }
        }
    }


// Ein Overlay ausgeben
    private synchronized void paint(Graphics2D g,CMGeom cmGeom,ArrayList<DebugPaintCommand> paintCommands) {

        for (DebugPaintCommand command: paintCommands) {
            switch (command.type) {
                 case 1: // fillSquare
                      g.setColor(command.color);
                      int[] pixel1=cmGeom.cm2pixel(command.cmX-command.cmSizeHalf,command.cmY-command.cmSizeHalf);
                      int[] pixel2=cmGeom.cm2pixel(command.cmX+command.cmSizeHalf,command.cmY+command.cmSizeHalf);
                      g.fillRect(pixel1[0],pixel2[1],pixel2[0]-pixel1[0],pixel1[1]-pixel2[1]);
                      break;

                 case 2: // fillCircle
                      g.setColor(command.color);
                      pixel1=cmGeom.cm2pixel(command.cmX-command.cmSizeHalf,command.cmY-command.cmSizeHalf);
                      pixel2=cmGeom.cm2pixel(command.cmX+command.cmSizeHalf,command.cmY+command.cmSizeHalf);
                      g.fillOval(pixel1[0],pixel2[1],pixel2[0]-pixel1[0],pixel1[1]-pixel2[1]);
                      break;

                 case 3: // cross
                      g.setColor(command.color);
                      g.setStroke(new BasicStroke((float)(2.0f*cmGeom.scale)));

                      pixel1=cmGeom.cm2pixel(command.cmX-command.cmSizeHalf,command.cmY-command.cmSizeHalf);
                      pixel2=cmGeom.cm2pixel(command.cmX+command.cmSizeHalf,command.cmY+command.cmSizeHalf);
                      g.drawLine(pixel1[0],pixel1[1],pixel2[0],pixel2[1]);
                      g.drawLine(pixel2[0],pixel1[1],pixel1[0],pixel2[1]);
                      break;

                 case 4: // drawText
                      g.setColor(command.color);
                      Font font=new Font("Serif",Font.BOLD,(int)Math.round(command.cmSize*1.4d*cmGeom.scale));
                      FontMetrics fontMet=g.getFontMetrics(font);
                      g.setFont(font); 
                      if (command.centered)
                          cmGeom.drawStringCentredCM(command.color,fontMet,command.text,command.cmX,command.cmY,command.clear);
                      else
                          cmGeom.drawStringLeftCM(command.color,fontMet,command.text,command.cmX,command.cmY,command.clear);
                      break;

                 case 5: // line
                      g.setColor(command.color);
                      g.setStroke(new BasicStroke((float)(2.0f*cmGeom.scale)));

                      pixel1=cmGeom.cm2pixel(command.cmX,command.cmY);
                      pixel2=cmGeom.cm2pixel(command.cmX2,command.cmY2);
                      g.drawLine(pixel1[0],pixel1[1],pixel2[0],pixel2[1]);
                      break;

                 case 6: // ellipse
                      g.setColor(command.color);

                      int[] pixel0=cmGeom.cm2pixel(command.cmX,command.cmY);
                      double alphaRad=command.rotAlpha*Math.PI/180;

                      g.translate(pixel0[0],pixel0[1]);
                      g.rotate(alphaRad);

                      pixel1=cmGeom.cm2pixel(command.cmX-command.cmRad1,command.cmY-command.cmRad2);
                      pixel2=cmGeom.cm2pixel(command.cmX+command.cmRad1,command.cmY+command.cmRad2);
                      g.fillOval(pixel1[0]-pixel0[0],pixel2[1]-pixel0[1],pixel2[0]-pixel1[0],pixel1[1]-pixel2[1]);

                      g.rotate(-alphaRad);
                      g.translate(-pixel0[0],-pixel0[1]);

                      break;

            }
        }
    }


}