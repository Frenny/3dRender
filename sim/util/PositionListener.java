package sim.util;

/**
* An object that is interested in the position report from the motion subsystem.
*/
public interface PositionListener {


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
    public void newPosition(long receiveTimeMillis,
                            double posX,double posY,double angle,
                            double tSpeed,double aSpeed,
                            double t2Speed,double a2Speed
                           ) throws Exception;


}