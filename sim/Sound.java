package sim;

import javax.sound.sampled.*;
import java.net.URL;

/**
* Class for sound output of the carbot simulator, e.g. for motor and collision sounds.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class Sound {
    // Motor-Sound-Ausgabe
    public static boolean shouldPlaySound=false; // Soll der MotorSound gespielt werden
    public static boolean canPlaySound=false;    // Kann der MotorSound gespielt werden
    
    private static boolean motorIsPlaying=false;       // Wird der Sound gerade gespielt (oder soll er gespielt werden?)   
    private static Clip motorSoundclip=null;
    private static FloatControl motorGainControl=null;

    private static boolean rumbleIsPlaying=false;       // Wird der Sound gerade gespielt (oder soll er gespielt werden?)   
    private static Clip rumbleSoundclip=null;
    private static FloatControl rumbleGainControl=null;


    public static void initSound() {
        // Initialisierung Motor-Sound
        try {
            URL url=Sound.class.getClassLoader().getResource("resources/carbotsound.wav");
//            System.out.println("Take motor sound from "+url);
            AudioInputStream audioStream = AudioSystem.getAudioInputStream(url);
            motorSoundclip=AudioSystem.getClip();
            motorSoundclip.open(audioStream);
            motorGainControl=(FloatControl)motorSoundclip.getControl(FloatControl.Type.MASTER_GAIN);
            
            url=Sound.class.getClassLoader().getResource("resources/rumble.wav");
//            System.out.println("Take rumble sound from "+url);
            audioStream = AudioSystem.getAudioInputStream(url);
            rumbleSoundclip=AudioSystem.getClip();
            rumbleSoundclip.open(audioStream);
            rumbleGainControl=(FloatControl)rumbleSoundclip.getControl(FloatControl.Type.MASTER_GAIN);            
            
            canPlaySound=true;
        }
        catch (Exception e) {
            System.out.println("Cannot initialize sounds: "+e);
            canPlaySound=false;
            e.printStackTrace();
        }
    }


    private static void nativeStartPlayMotorSound() {
         motorSoundclip.loop(Clip.LOOP_CONTINUOUSLY);
    }
    

    private static void nativeStartPlayRumbleSound() {
         rumbleSoundclip.loop(Clip.LOOP_CONTINUOUSLY);
    }    
    

    private static void nativeStopPlayMotorSound() {
        if (motorSoundclip.isRunning())
            motorSoundclip.stop();
    }
    
    
    private static void nativeStopPlayRumbleSound() {
        if (rumbleSoundclip.isRunning())
            rumbleSoundclip.stop();
    }


    public static void startMotorSound() {
        if (motorIsPlaying)
            return;
        motorIsPlaying=true;
        if (canPlaySound & shouldPlaySound) {
            nativeStartPlayMotorSound();
        }
    }
    
    
    public static void startRumbleSound() {
        if (rumbleIsPlaying)
            return;
        rumbleIsPlaying=true;
        if (canPlaySound & shouldPlaySound) {
            nativeStartPlayRumbleSound();
        }
    }    

    public static void stopMotorSound() {
        if (!motorIsPlaying)
            return;

        motorIsPlaying=false;
        if (canPlaySound & shouldPlaySound) {
            nativeStopPlayMotorSound();
        }
    }
    
    
    public static void stopRumbleSound() {
        if (!rumbleIsPlaying)
            return;

        rumbleIsPlaying=false;
        if (canPlaySound & shouldPlaySound) {
            nativeStopPlayRumbleSound();
        }
    }    


    public static void setSoundOnOff(boolean on) {
        if (shouldPlaySound==on)
            return;

        shouldPlaySound=on;
        if (motorIsPlaying) {
            if (shouldPlaySound) {
                nativeStartPlayMotorSound();
            }
            else {
                nativeStopPlayMotorSound();
            }
        }
        if (rumbleIsPlaying) {
            if (shouldPlaySound) {
                nativeStartPlayRumbleSound();
            }
            else {
                nativeStopPlayRumbleSound();
            }
        }        
    }


    public static void setVolume(int volIndex) {   // 0: laut, 9: leise
        if (canPlaySound & shouldPlaySound) {
            motorGainControl.setValue(-volIndex*4.0f);
            rumbleGainControl.setValue(-volIndex*4.0f);
        }
    }
}