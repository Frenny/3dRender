package sim.util;

import java.net.Socket;
import java.net.InetAddress;
import java.io.DataInputStream;
import java.io.IOException;

import robotinterface.util.DebugPainter;
import robotinterface.util.DebugOut;


/**
* Client to connect to the debug painting and debug out server, i.e. the Carbot.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class RemoteDebugClient extends Thread {

    private DataInputStream in;
    private DebugPainter painter;
    private DebugOut debugOut;

    public RemoteDebugClient(DataInputStream in,DebugPainter painter, DebugOut debugOut) {
        this.painter=painter;
        this.debugOut=debugOut;
        this.in=in;
        System.out.println("Remote DebugPainter established to remote host");
        start();
    }


    public void run(){
        try {
            while (true) {
                short cmd=in.readShort();

                switch (cmd) {
                     case -1: //-1=println an debugout
                          String line=in.readUTF();
                          debugOut.println(line);
                          break;
                          
                     case 1: // 1=setOverlay
                          String overlay=in.readUTF();
                          painter.setOverlay(overlay);
                          break;
                          
                     case 2: // 2=clear
                          painter.clear();
                          break;
                          
                     case 3: // 3=paint
                          painter.paint();
                          break;
                          
                     case 4: // 4=fillSquare
                          double cmX=in.readDouble();
                          double cmY=in.readDouble();
                          double cmSize=in.readDouble();
                          int red=in.readInt();
                          int green=in.readInt();
                          int blue=in.readInt();
                          int alpha=in.readInt();
                          painter.fillSquare(cmX,cmY,cmSize,red,green,blue,alpha);
                          break;
                          
                     case 5: // 5=fillCircle
                          cmX=in.readDouble();
                          cmY=in.readDouble();
                          cmSize=in.readDouble();
                          red=in.readInt();
                          green=in.readInt();
                          blue=in.readInt();
                          alpha=in.readInt();
                          painter.fillCircle(cmX,cmY,cmSize,red,green,blue,alpha);
                          break;
                          
                     case 6: // 6=drawCross
                          cmX=in.readDouble();
                          cmY=in.readDouble();
                          cmSize=in.readDouble();
                          red=in.readInt();
                          green=in.readInt();
                          blue=in.readInt();
                          alpha=in.readInt();
                          painter.drawCross(cmX,cmY,cmSize,red,green,blue,alpha);
                          break;
                          
                     case 7: // 7=drawText
                          cmX=in.readDouble();
                          cmY=in.readDouble();
                          String text=in.readUTF();
                          boolean centered=in.readBoolean();
                          boolean clear=in.readBoolean();
                          double cmHeight=in.readDouble();
                          red=in.readInt();
                          green=in.readInt();
                          blue=in.readInt();
                          alpha=in.readInt();
                          painter.drawText(cmX,cmY,text,centered,clear,cmHeight,red,green,blue,alpha);
                          break;
                          
                     case 8: // 8=drawLine
                          cmX=in.readDouble();
                          cmY=in.readDouble();
                          double cmX2=in.readDouble();
                          double cmY2=in.readDouble();
                          red=in.readInt();
                          green=in.readInt();
                          blue=in.readInt();
                          alpha=in.readInt();
                          painter.drawLine(cmX,cmY,cmX2,cmY2,red,green,blue,alpha);
                          break;
                          
                     case 9: // 9=fillEllipse  
                          cmX=in.readDouble();
                          cmY=in.readDouble();
                          double cmRad1=in.readDouble();
                          double cmRad2=in.readDouble();
                          double rotAlpha=in.readDouble();
                          red=in.readInt();
                          green=in.readInt();
                          blue=in.readInt();
                          alpha=in.readInt();
                          painter. fillEllipse(cmX,cmY,cmRad1,cmRad2,rotAlpha,red,green,blue,alpha);
                          break;
                          
                     default: 
                          System.out.println("Remote DebugPainter terminated (illegal cmd "+cmd+")");
                          return;

                }

            }
        }
        catch (IOException e) {
            System.out.println("Remote DebugPainter terminated ("+e.toString()+")");
        }
    }

}