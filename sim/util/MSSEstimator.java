package sim.util;

import robotinterface.util.GeomUtil;

/**
* Estimates a position for timestamps that are not covered by explicit position messages from the Motion Subsystem. Such as position is computed according to the assumption,
* the Carbot drives steadily between two positions (e.g. foreward, bow or turn in place). There are two cases:
* <ul>
*  <li>Case <i>past</i>: The timestamp is between two explicity position messages in the past: then the position is estimated using the two adjacent position messages.</li>
*  <li>Case <i>future</i>: The timestamp is after the newest position message: then the position is estimated using the newest position message together with the newest driving and angle speeds.</li>
* </ul>
* <p>For the first case, this class stores old position messages up to a certain time in the past. This time depends on the number of messages, i.e. there is not absolute time range.</p>
* Note that the <i>future</i> case means &quot;after the newest message&quot;. That can also be in the past, i.e. after the newest message, but before the current time stamp.
* In other words, <i>past</i> and <i>future</i> are noted according to the last position message, <i>not</i> according to the current time.
*/
public class MSSEstimator implements PositionListener {

    private int bufferSize=-1;

    private long[] receiveTimeBuffer=null;
    private double[] xBuffer=null;
    private double[] yBuffer=null;
    private double[] angBuffer=null;  // Rechen-Winkel (wie nav, d.h. radians, 0=x-Achse, counter-clockwise)

    private int lastSetBufEntry=-1;
    private int nextFreeBufEntry=0;
    private int firstSetBufEntry=-1;


    private long lastReceiveTime=-1;
    private double lastX=Double.NaN;
    private double lastY=Double.NaN;
    private double lastAng=Double.NaN;
    private double lastT2Speed=Double.NaN;
    private double lastA2Speed=Double.NaN;


/**
* Instantiates an estimator.
* @param numOfStoredMessages the max. number of stored messages for the past estimation
*/
    public MSSEstimator(int numOfStoredMessages) {

        if (numOfStoredMessages<5)
            throw new IllegalArgumentException("numOfStoredMessages too low");

        bufferSize=numOfStoredMessages;
        receiveTimeBuffer=new long[numOfStoredMessages];
        xBuffer=new double[numOfStoredMessages];
        yBuffer=new double[numOfStoredMessages];
        angBuffer=new double[numOfStoredMessages];
    }


/**
* The motion subsystem reports a new position.
* @param receiveTimeMillis system time when receiving the position measurement
* @param posX current X cm
* @param posY current y cm
* @param angle orientation degrees
* @param tSpeed last measured driving speed
* @param aSpeed last measured angle speed
* @param t2Speed future driving speed
* @param a2Speed future angle speed
* @throws Exception whatever happens
*/
    public synchronized void newPosition(long receiveTimeMillis,
                            double posX,double posY,double angle,
                            double tSpeed,double aSpeed,
                            double t2Speed,double a2Speed
                           ) throws Exception {


        angle=GeomUtil.mssAngle2NavAngle(angle);  // Erst in die "Rechen-Winkel" umwandeln

        lastReceiveTime=receiveTimeMillis;
        lastX=posX;
        lastY=posY;
        lastAng=angle;
        lastT2Speed=t2Speed/1000.0d;                  // Damit cm/ms
        lastA2Speed=-a2Speed*Math.PI/180000.0d;        // Damit radians/ms


        receiveTimeBuffer[nextFreeBufEntry]=receiveTimeMillis;
        xBuffer[nextFreeBufEntry]=posX;
        yBuffer[nextFreeBufEntry]=posY;
        angBuffer[nextFreeBufEntry]=angle;


        lastSetBufEntry=nextFreeBufEntry;

        if (firstSetBufEntry<0)
            firstSetBufEntry=0;
        else if (firstSetBufEntry==nextFreeBufEntry)
            firstSetBufEntry=(firstSetBufEntry+1) % bufferSize;

        nextFreeBufEntry=(nextFreeBufEntry+1) % bufferSize;
    }


/**
* Estimate a position (future or past).
* @param timeMillis the time for which the position should be estimated
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if the timestamp is before the earliest position message or there was no position message at all until now.
*/
    public double[] estimatePosition(long timeMillis) {
        return estimatePosition(timeMillis, true,true,null);
    }


/**
* Estimate a position of the past.
* @param timeMillis the time for which the position should be estimated
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if one of these cases occur<ul>
*           <li>no position message at all until now</li>
*           <li>the timestamp is before the earliest position message</li>
*           <li>the timestamp is after the newest position message</li>
*         </ul>
*/
    public double[] estimatePastPosition(long timeMillis) {
        return estimatePosition(timeMillis,true,false,null);
    }


/**
* Estimate a position of the future.
* @param timeMillis the time for which the position should be estimated
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if one of these cases occur<ul>
*           <li>no position message at all until now</li>
*           <li>the timestamp is before the newest position message</li>
*         </ul>
*/
    public double[] estimateFuturePosition(long timeMillis) {
        return estimatePosition(timeMillis,false,true,null);
    }


/**
* Estimate a position. The caller configure, if for future, past, or both. Moreover the caller can ask for the respective times stored in the message buffer.
* @param timeMillis the time for which the position should be estimated
* @param past can the timestamp be before the last explicit position message
* @param future can the timestamp be after the last explicit position message
* @param storedTimes the caller can pass a long[2] which is filled with timestamps: [0] earliest position message [1] newest position message;
*                    set to {-1, -1} if there was not position message at all until now.
* @return array of three with [0]: x, [1]: y, [2]: orientation in MSS angle (degrees, 0&deg; points in y-direction, positive angles go clockwise).
*         The result also maybe null, if one of these cases occur<ul>
*           <li>no position message at all until now</li>
*           <li>the timestamp is before the earliest position message</li>
*           <li>the timestamp is before the newest position message and past==false</li>
*           <li>the timestamp is after the newest position message and future==false</li>
*         </ul>
*/
    public synchronized double[] estimatePosition(long timeMillis,boolean past,boolean future,long[] storedTimes) {
        if (storedTimes!=null && storedTimes.length!=2)
            throw new IllegalArgumentException("storedTimes must be null or long[2]");

        if (lastReceiveTime<0) {  // Noch nie eine Meldung empfangen
            if (storedTimes!=null) {
                storedTimes[0]=-1;
                storedTimes[1]=-1;
            }
            return null; 
        }


        if (storedTimes!=null) {     // Die relevanten Zeiten zurückgeben
            storedTimes[0]=receiveTimeBuffer[firstSetBufEntry];
            storedTimes[1]=lastReceiveTime;
        }


        if (timeMillis>=lastReceiveTime) {   // Fall "Zukunft"
            if (!future)
                return null;
            double[] result=GeomUtil.extrapolatePose(lastX,lastY,lastAng,
                                            lastT2Speed,lastA2Speed,
                                            timeMillis-lastReceiveTime);
            result[2]=GeomUtil.navAngle2MSSAngle(result[2]);  // Ergebnis soll in MSS-Winkeln sein
            return result;
        }

        // Fall "Vergangenheit"

        if (timeMillis<receiveTimeBuffer[firstSetBufEntry])   // Vor der ersten gespeicherten Meldung
            return null; 

        int from=firstSetBufEntry;
        int to=lastSetBufEntry;
        int range=(to+bufferSize-from) % bufferSize;

        // Erst einmal über Binärsuche die angrenzenden Messungen im Buffer finden
        while (range>1) {
            int mid=(from+range/2) % bufferSize;
            if (timeMillis>=receiveTimeBuffer[mid])
                from=mid;
            else {
                to=mid;
            }
            range=(to+bufferSize-from) % bufferSize;
        }


        double[] result=GeomUtil.interpolatePose(xBuffer[from], yBuffer[from], angBuffer[from],
                                                 xBuffer[to], yBuffer[to], angBuffer[to],
                                                 (timeMillis-receiveTimeBuffer[from])/(receiveTimeBuffer[to]-receiveTimeBuffer[from]));

        result[2]=GeomUtil.navAngle2MSSAngle(result[2]);  // Ergebnis soll in MSS-Winkeln sein
        return result;
    }


}