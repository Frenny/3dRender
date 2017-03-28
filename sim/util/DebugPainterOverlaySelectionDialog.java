package sim.util;


import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import javax.swing.JFrame;
import javax.swing.JDialog;
import javax.swing.JButton;
import javax.swing.JCheckBox;

import java.awt.event.WindowListener;
import java.awt.event.WindowEvent;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.ChangeEvent;

import java.util.ArrayList;
import java.util.HashSet;

import robotinterface.Robot;

/**
* Dialog to select debug painting overlays.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class DebugPainterOverlaySelectionDialog extends JDialog implements ActionListener,ChangeListener,WindowListener {

    private static DebugPainterOverlaySelectionDialog openedDialog=null;

    private JButton okButton=null;

    private ArrayList<JCheckBox> checkboxes=null;
    private ArrayList<String> overlays=null;


    public static void openDialog(JFrame owner) {
        if (openedDialog==null)
            openedDialog=new DebugPainterOverlaySelectionDialog(owner);
    }


    private DebugPainterOverlaySelectionDialog(JFrame owner) {
        super(owner,"Select overlays");
        initDialog();
        setLocationRelativeTo(owner);
    }


    private void initDialog() {

        GridBagLayout gridbag=new GridBagLayout();
        GridBagConstraints c=new GridBagConstraints();

        setLayout(gridbag);  

        c.fill=GridBagConstraints.HORIZONTAL;


        checkboxes=new ArrayList<>();
        overlays=((SimDebugPainter)Robot.debugPainter).getOverlays();
        HashSet<String> disabled=((SimDebugPainter)Robot.debugPainter).getOverlayDisabled();
        for (int i=0;i<overlays.size();i++) {
            String overlayName=overlays.get(i);
            JCheckBox check=new JCheckBox(overlayName);
            c.gridx=0;
            c.gridy=i;
            c.weightx=0.0;
            c.weighty=0.0;
            c.gridwidth=1;
            c.gridheight=1;
            gridbag.setConstraints(check,c);
            check.setSelected(!disabled.contains(overlayName));
            add(check);
            check.addChangeListener(this);
            checkboxes.add(check);
        }


        okButton=new JButton("OK");
        c.gridx=0;
        c.gridy=checkboxes.size()+1;
        c.weightx=0.0;
        c.weighty=0.0;
        c.gridwidth=1;
        c.gridheight=1;
        gridbag.setConstraints(okButton,c);
        add(okButton);
        okButton.addActionListener(this);

        addWindowListener(this);
        setVisible(true);
//        doLayout();
        pack();
        
    }


    public void stateChanged(ChangeEvent e) {
    
        if (overlays.size()!=checkboxes.size()) {   // Achtung: im Hintergrund wurden die overlays vom Robot-Controller verändert -> alles disabled!
            for (int i=0;i<checkboxes.size();i++)
                checkboxes.get(i).setEnabled(false);
            return;
        }
    
        HashSet<String> diabledOverlays=new HashSet<>();
        for (int i=0;i<overlays.size();i++) {
            if (!checkboxes.get(i).isSelected()) {
                diabledOverlays.add(overlays.get(i));
            }
        }
        ((SimDebugPainter)Robot.debugPainter).setOverlayDisabled(diabledOverlays);
    }


    public void actionPerformed(ActionEvent e) {
        if (e.getSource()==okButton) {
            openedDialog=null;
            setVisible(false);
        }
    }


    public void windowDeactivated(WindowEvent e) {
    }


    public void windowActivated(WindowEvent e) {
    }


    public void windowIconified(WindowEvent e) {
    }


    public void windowDeiconified(WindowEvent e) {
    }


    public void windowClosing(WindowEvent e) {
        openedDialog=null;
    }


    public void windowOpened(WindowEvent e) {
    }


    public void windowClosed(WindowEvent e) {
    }

}