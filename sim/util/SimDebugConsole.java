package sim.util;


import javax.swing.JFrame;
import javax.swing.JDialog;
import javax.swing.JTextArea;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import javax.swing.JScrollPane;
import javax.swing.JScrollBar;
import javax.swing.text.BadLocationException;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.IOException;

import java.awt.event.WindowListener;
import java.awt.event.WindowEvent;
import java.awt.Toolkit;
import java.awt.Point;
import java.awt.Dimension;

//import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ArrayBlockingQueue;

import robotinterface.util.DebugOut;
import sim.CarbotSim;

/**
* An implementaion of the DebugOut that supports debug text on a simulator's console frame.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class SimDebugConsole extends JDialog implements WindowListener,DebugOut {
    private final static int MAX_LINE_CNT=100;

    private final static String DEBUG_OUT_FILE="console.out";


    private CarbotSim carbotSim;
    private int posX;
    private int posY;
    private int sizeX;
    private int sizeY;

    private PrintWriter consolePrintWriter=null;

    private JScrollPane resultScrollPane=null;
    private JTextArea textArea=null;


    private boolean showed=false;

//    private ConcurrentLinkedQueue<String> queuedLines;
    private BlockingQueue<String> queuedLines;
    private String[] displayedLines;
    private int nextFreeLine;
    private int firstLine;
    private int lineCnt;



/**
* Create the debug console without opening it. From now, debug-out messages are buffered.
* @param carbotSim main simulation window
*/
    public SimDebugConsole(CarbotSim carbotSim) {
        super(carbotSim,"Console");

        this.carbotSim=carbotSim;
//        queuedLines=new ConcurrentLinkedQueue<>();
        queuedLines=new ArrayBlockingQueue<>(MAX_LINE_CNT*2);
        displayedLines=new String[MAX_LINE_CNT];
        nextFreeLine=0;
        firstLine=0;
        lineCnt=0;

        try {
            consolePrintWriter=new PrintWriter(DEBUG_OUT_FILE);
        }
        catch (IOException e) {
            System.out.println("Cannot redirect debug out to "+DEBUG_OUT_FILE+" ("+e.toString()+")");
        }

        Thread consoleThread=new Thread(new Runnable() {
                public void run() {
                    while (true) {

//                        doWait();
//                        while (!queuedLines.isEmpty()) {
//                            String line=queuedLines.poll();
                            String line=null;

                            try {
                                Thread.sleep(2);  // Komischerweise notwendig, damit Debug-Nachrichten sich nicht überholen (ja, echt!) - obwohl die Queue Thread-safe sein soll
                                line=queuedLines.take();
                            }
                            catch (InterruptedException e) {
                                continue;
                            }

                            if (consolePrintWriter!=null) {
                                consolePrintWriter.println(line);
                                consolePrintWriter.flush();
                            }

                            if (showed)
                                addLine(line);
                            displayedLines[nextFreeLine]=line;
                            nextFreeLine=(nextFreeLine+1) % MAX_LINE_CNT;
                            lineCnt++;
                            if (lineCnt>MAX_LINE_CNT) {
                                lineCnt=MAX_LINE_CNT;
                                firstLine=nextFreeLine;
                            }
                        }
//                    }
                }
           }
        );
        consoleThread.start();

    }

/**
* Sets the relative position and window size.
* @param posX position x relative to parent
* @param posY position y relative to parent
* @param sizeX window's width
* @param sizeY window's height
*/
    public void setScreenConfiguration(int posX,int posY,int sizeX,int sizeY) {
        this.posX=posX;
        this.posY=posY;
        this.sizeX=sizeX;
        this.sizeY=sizeY;

        setLocation(new Point(posX,posY));
        setSize(sizeX,sizeY);
    }


/**
* Returns the current screen configuration.
* @return array of 4 with pos x, y; size x, y
*/
    public int[] getScreenConfiguration() {
        Point p=getLocation();
        Dimension d=getSize();
        return new int[]{p.x,p.y,d.width,d.height};
    }


/**
* Open the debug console.
*/
    public void showConsole() {
        initUI();

        for (int i=0;i<lineCnt;i++) {
            String line=displayedLines[(i+firstLine) % MAX_LINE_CNT];
            textArea.append(line+"\n");
        }
        textArea.setCaretPosition(textArea.getDocument().getLength());

        setVisible(true);
        showed=true;
    }


//    private synchronized void doWait() {
//        try {
//            wait(); 
//        }
//        catch (InterruptedException e) {}
//    }


/**
* Close the debug console. Messages are still buffered in background.
*/
    public void hideConsole() {
        setVisible(false);
        showed=false;
    }


/**
* Transmits a text line to the console out.
* @param line text line
*/
    public synchronized void println(String line) {
        queuedLines.add(line);
//        notify();
    }



/**
* Prints the stack trace related to the exception to console out.
* @param e an exception 
*/
    public void printStackTrace(Exception e) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        String swStr=sw.toString();
        println(swStr);
        System.out.println(swStr);
    }


    private void addLine(String line) {
        textArea.append(line+"\n");

        int lineCnt=textArea.getLineCount();
        if (lineCnt>MAX_LINE_CNT) {
            try {
                textArea.replaceRange("",0,textArea.getLineStartOffset(lineCnt-MAX_LINE_CNT));
            }
            catch (BadLocationException e) {
                e.printStackTrace();
            }
        }
        textArea.setCaretPosition(textArea.getDocument().getLength());   // Nach unten scrollen

//        JScrollBar vertical= resultScrollPane.getVerticalScrollBar();  // Geht auch, um nach unten zu scrollen
//        vertical.setValue(vertical.getMaximum() );
    }


    private void initUI() {
        try {
            setIconImage(Toolkit.getDefaultToolkit().getImage(CarbotSim.class.getClassLoader().getResource("resources/console32x32.png")));
        }
        catch (Exception e) {}

//        setAlwaysOnTop(true);

        if (resultScrollPane!=null)
            remove(resultScrollPane);

        GridBagLayout gridbag=new GridBagLayout();
        GridBagConstraints c=new GridBagConstraints();

        setLayout(gridbag);  

        textArea=new JTextArea();  // 8,20
        textArea.setEditable(false);
        textArea.setToolTipText("Debug Out console");


        resultScrollPane=new JScrollPane(textArea);
        c.gridx=0;
        c.gridy=5;
        c.weightx=0.1;
        c.weighty=1.0;
        c.gridwidth=2;
        c.gridheight=1;
        c.fill=GridBagConstraints.BOTH;
        c.anchor=GridBagConstraints.CENTER;
        gridbag.setConstraints(resultScrollPane,c);
        add(resultScrollPane);

        addWindowListener(this);
        doLayout();
    }



    public void windowActivated(WindowEvent e) {
    }

    public void windowClosed(WindowEvent e) {
    }

    public void windowClosing(WindowEvent e) {
        hideConsole();
        carbotSim.setConsoleCheck(false);
    }

    public void windowDeactivated(WindowEvent e) {
    }

    public void windowDeiconified(WindowEvent e) {
    }

    public void windowIconified(WindowEvent e) {
    }

    public void windowOpened(WindowEvent e) {
    }

}