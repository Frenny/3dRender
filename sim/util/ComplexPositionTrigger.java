package sim.util;

/**
* An object used to inform other objects whenever the position changed.
* The reason why the change not simply calls the respective objects: we need a decoupling
* from the change trigger that runs inside the receive method for async messages from the motion subsystem, from the 
* processing methods that take the new position.
* @author J&ouml;rg Roth (<a href="mailto:Joerg.Roth@Wireless-earth.org">Joerg.Roth@Wireless-earth.org</a>)
*/
public class ComplexPositionTrigger extends Thread {


/** Timing type: triggered by position messages */
    public final static int TRIGGERED=1;

/** Timing type: equidistant (ignoring position messages) */
    public final static int EQUIDISTANT=2;

/** Timing type: synchronized with position messages */
    public final static int SYNCHRONIZED=3;


    private long lastReceiveTimeMillis=-1;
    private long lastLastAsyncPeriodic=-1;
    private boolean lastAdditionalPosMessage=false;

    private double lastPosX=Double.NaN;
    private double lastPosY=Double.NaN;
    private double lastAngle=Double.NaN;

    private ComplexPositionListener[] toTrigger=null;
    private boolean triggered=false;
    private long[] lastExecTime=null;
    private boolean infiniteWait=false; // Steht der Thread auf auf dem wait() ohne Timeout?
    private long lastTriggerTime=-1;    // Wann wurde das letzte Mal der Trigger ausgeführt (alle Typen)

    private MSSEstimator mssEstimator=null;
    private long deltaT=-1;
    private int timingType=TRIGGERED;


/**
* Change the timing configuration.
* @param timingType one of TRIGGERED, EQUIDISTANT, SYNCHRONIZED
* @param deltaT for EQUIDISTANT, SYNCHRONIZED: the cycle time in ms. For TRIGGERED must be -1
*/
    public void setTiming(int timingType,long deltaT) {
        if (timingType<TRIGGERED  || timingType>SYNCHRONIZED)
            throw new IllegalArgumentException("timingType="+timingType+" is out of accepted values");

        if (timingType==TRIGGERED && deltaT>=0)
            throw new IllegalArgumentException("no deltaT>0 allowed for timingType=TRIGGERED");

        if (timingType==EQUIDISTANT || timingType==SYNCHRONIZED) {
            if (deltaT<0)
               throw new IllegalArgumentException("deltaT>0 required for timingType=EQUIDISTANT or SYNCHRONIZED");
            if (deltaT<200)
               throw new IllegalArgumentException("deltaT="+deltaT+" too small");
        }

        this.deltaT=deltaT;
        this.timingType=timingType;
    }


/**
* Creates an object that can inform another object whenever the position changes.
* The other object's newPosition() method is called in this case.
* @param toTrigger a listener object
* @param mssEstimator the position estimator
*/
    public ComplexPositionTrigger(MSSEstimator mssEstimator,ComplexPositionListener toTrigger) {
        this.toTrigger=new ComplexPositionListener[]{toTrigger};
        this.mssEstimator=mssEstimator;
        lastExecTime=new long[1];
        start();
    }


/**
* Creates an object that can inform other objects whenever the position changes.
* The other objects' newPosition() method is called in this case.
* @param toTrigger a list of listener objects
* @param mssEstimator the position estimator
*/
    public ComplexPositionTrigger(MSSEstimator mssEstimator,ComplexPositionListener[] toTrigger) {
        this.toTrigger=toTrigger;
        this.mssEstimator=mssEstimator;
        lastExecTime=new long[toTrigger.length];
        start();
    }


/**
* The current position changed - as a consequence the respective registered objects are informed.
* However, this method returns immediately without waiting on the registered objects.
* @param receiveTimeMillis system time when receiving the position measurement
* @param lastAsyncPeriodic the periodic time measured by the last to async messages
* @param additionalPosMessage true if the position message was &quot;additional&quot;, i.e. as a result of new or terminating motion command
* @param posX current X cm
* @param posY current y cm
* @param angle orientation degrees
*/
    public synchronized void triggerNewPosition(long receiveTimeMillis,
                                                long lastAsyncPeriodic,
                                                boolean additionalPosMessage,
                                                double posX,double posY,double angle
                                               ) {

        if (triggered && (timingType==TRIGGERED || timingType==SYNCHRONIZED && !additionalPosMessage)) {
            System.out.println("WARNING: new position trigger, but last trigger ("+(receiveTimeMillis-lastReceiveTimeMillis)+"ms ago) was not processed!");
            System.out.println("Position triggers execution times:");
            for (int i=0;i<toTrigger.length;i++) {
                System.out.println("- "+toTrigger[i].getClass().getName()+": "+lastExecTime[i]+"ms");
            }
        }

        lastReceiveTimeMillis=receiveTimeMillis;
        lastLastAsyncPeriodic=lastAsyncPeriodic;
        lastAdditionalPosMessage=additionalPosMessage;
        lastPosX=posX;
        lastPosY=posY;
        lastAngle=angle;

        switch (timingType) {
            case TRIGGERED:
                triggered=true;
                notify();
                break;

            case SYNCHRONIZED:

                if (infiniteWait) {
                    triggered=true;
                    notify();
                }
                else {
                    long currentTime=System.currentTimeMillis();

                    if (lastTriggerTime<0)
                        lastTriggerTime=currentTime;

                    long sinceLastTrigger=currentTime-lastTriggerTime;
                    long sinceLastTrigger2=currentTime+lastLastAsyncPeriodic-lastTriggerTime;

                    long distToDelta=Math.abs(sinceLastTrigger-deltaT);
                    long distToDelta2=Math.abs(sinceLastTrigger2-deltaT);

                    if (distToDelta<deltaT/2 && (distToDelta<distToDelta2 || sinceLastTrigger2>deltaT*95/100)) {
                        triggered=true;
                        notify();
                    }
                }
                break;

            case EQUIDISTANT:
                if (infiniteWait) {
                    triggered=true;
                    notify();
                }
                break;
        }
    }


/**
* The thread's running method.
*/
    public void run() {
        while (true) {

            long lastReceiveTimeMillis_;
            boolean lastAdditionalPosMessage_;
            double lastPosX_;
            double lastPosY_;
            double lastAngle_;

            synchronized(this) {
                if (!triggered) {
                    try {
                        if (timingType==TRIGGERED) {
                            infiniteWait=true;
                            wait();
                            infiniteWait=false;
                        }
                        else if (timingType==SYNCHRONIZED) {
                            long currentTime=System.currentTimeMillis();
                            long distToNextRegular=lastReceiveTimeMillis+lastLastAsyncPeriodic-currentTime;
                            long distToNextRegular2=lastReceiveTimeMillis+2*lastLastAsyncPeriodic-currentTime;

                            long distToDelta=Math.abs(distToNextRegular-deltaT);
                            long distToDelta2=Math.abs(distToNextRegular2-deltaT);

                            if (distToDelta<deltaT/2 && distToDelta<distToDelta2) {  // Der reguläre Trigger würde in das nächste Intervall reinfallen
                                wait(deltaT*3/2);
                            }
                            else {
                                wait(deltaT);

                            }
                        }
                        else {
                            Thread.sleep(deltaT);
                        }
                    }
                    catch (InterruptedException e) {};
                }

                lastReceiveTimeMillis_=lastReceiveTimeMillis;
                lastAdditionalPosMessage_=lastAdditionalPosMessage;
                lastPosX_=lastPosX;
                lastPosY_=lastPosY;
                lastAngle_=lastAngle;
            }

            lastTriggerTime=System.currentTimeMillis();
            double[] estimatedPos=null; // Schätzung, wenn erforderlich

            if (!triggered) {
                estimatedPos=mssEstimator.estimateFuturePosition(lastTriggerTime);
            }


            for (int i=0;i<toTrigger.length;i++) {
                ComplexPositionListener t=toTrigger[i];

                long startTime=System.currentTimeMillis();
                try {
                    if (triggered) {
                        t.newPosition(lastReceiveTimeMillis_, 
                                      (lastAdditionalPosMessage_ ? ComplexPositionListener.ADDITIONAL : ComplexPositionListener.REGULAR),
                                      lastPosX_,lastPosY_,lastAngle_
                                     );
                    }
                    else {
                        t.newPosition(lastTriggerTime,
                                      ComplexPositionListener.ESTIMATED,
                                      estimatedPos[0],estimatedPos[1],estimatedPos[2]
                                     );
                    }
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