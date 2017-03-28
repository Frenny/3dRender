package sim.util;

/**
* An object that is interested in the position report from the motion subsystem.
* The complex listener also listens to estimated positions.
*/
public interface ComplexPositionListener {

/** The new position was a result of a regular position message by the Motion Subsystem, i.e. a periodic message. */
    public final static int REGULAR=1;

/** The new position was a result of an additional position message by the Motion Subsystem, i.e. because of motion command start or termination. */
    public final static int ADDITIONAL=2;

/** The new position was a result of an estimation. */
    public final static int ESTIMATED=3;



/**
* The motion subsystem reports a new position.
* @param receiveTimeMillis system time when receiving the position measurement or when estimating the position
* @param type position measurement type, see constants
* @param posX current X cm
* @param posY current y cm
* @param angle orientation degrees
* @throws Exception whatever happens
*/
    public void newPosition(long receiveTimeMillis,
                            int type,
                            double posX,double posY,double angle
                           ) throws Exception;


}