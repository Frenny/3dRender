package sim.util;

/**
* An object used to inform other objects whenever the position changed.
* The reason why the change not simply calls the respective objects: we need a decoupling
* from the change trigger that runs inside the receive method for async messages from the motion subsystem, from the 
* processing methods that take the new position.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class PositionTrigger extends Thread {

    private long lastReceiveTimeMillis=-1;
    private double lastPosX=Double.NaN;
    private double lastPosY=Double.NaN;
    private double lastAngle=Double.NaN;
    private double lastTSpeed=Double.NaN;
    private double lastASpeed=Double.NaN;
    private double lastT2Speed=Double.NaN;
    private double lastA2Speed=Double.NaN;

    private PositionListener[] toTrigger=null;
    private boolean triggered=false;
    private long[] lastExecTime=null;


/**
* Creates an object that can inform another object whenever the position changes.
* The other object's newPosition() method is called in this case.
* @param toTrigger a listener object
*/
    public PositionTrigger(PositionListener toTrigger) {
        this.toTrigger=new PositionListener[]{toTrigger};
        lastExecTime=new long[1];
        start();
    }


/**
* Creates an object that can inform other objects whenever the position changes.
* The other objects' newPosition() method is called in this case.
* @param toTrigger a list of listener objects
*/
    public PositionTrigger(PositionListener[] toTrigger) {
        this.toTrigger=toTrigger;
        lastExecTime=new long[toTrigger.length];
        start();
    }


/**
* The current position changed - as a consequence the respective registered objects are informed.
* However, this method returns immediately without waiting on the registered objects.
* @param receiveTimeMillis system time when receiving the position measurement
* @param posX current X cm
* @param posY current y cm
* @param angle orientation degrees
* @param tSpeed last measured driving speed
* @param aSpeed last measured angle speed
* @param t2Speed future driving speed
* @param a2Speed future angle speed
*/
    public synchronized void triggerNewPosition(long receiveTimeMillis,
                                                double posX,double posY,double angle,
                                                double tSpeed,double aSpeed,
                                                double t2Speed,double a2Speed
                                               ) {

        if (triggered) {
            System.out.println("WARNING: new position trigger, but last trigger ("+(receiveTimeMillis-lastReceiveTimeMillis)+"ms ago) was not processed!");
            System.out.println("Position triggers execution times:");
            for (int i=0;i<toTrigger.length;i++) {
                System.out.println("- "+toTrigger[i].getClass().getName()+": "+lastExecTime[i]+"ms");
            }
        }

        lastReceiveTimeMillis=receiveTimeMillis;
        lastPosX=posX;
        lastPosY=posY;
        lastAngle=angle;
        lastTSpeed=tSpeed;
        lastASpeed=aSpeed;
        lastT2Speed=t2Speed;
        lastA2Speed=a2Speed;

        triggered=true;
        notify();
    }


/**
* The thread's running method.
*/
    public void run() {
        while (true) {

            long lastReceiveTimeMillis_;
            double lastPosX_;
            double lastPosY_;
            double lastAngle_;
            double lastTSpeed_;
            double lastASpeed_;
            double lastT2Speed_;
            double lastA2Speed_;

            synchronized(this) {
                if (!triggered) {
                    try {
                        wait();
                    }
                    catch (InterruptedException e) {};
                }

                lastReceiveTimeMillis_=lastReceiveTimeMillis;
                lastPosX_=lastPosX;
                lastPosY_=lastPosY;
                lastAngle_=lastAngle;
                lastTSpeed_=lastTSpeed;
                lastASpeed_=lastASpeed;
                lastT2Speed_=lastT2Speed;
                lastA2Speed_=lastA2Speed;
            }

            for (int i=0;i<toTrigger.length;i++) {
                PositionListener t=toTrigger[i];

                long startTime=System.currentTimeMillis();
                try {
                    t.newPosition(lastReceiveTimeMillis_,
                                  lastPosX_,lastPosY_,lastAngle_,
                                  lastTSpeed_,lastASpeed_,lastT2Speed_,lastA2Speed_
                                 );
                }
                catch (Exception e) {
                    System.out.println("Error triggering an object due to a position change of class "+t.getClass().getName()+":");
                    e.printStackTrace();
                }
                lastExecTime[i]=System.currentTimeMillis()-startTime;
            }
            synchronized(this) {
                triggered=false;
            }
        }
    }

}