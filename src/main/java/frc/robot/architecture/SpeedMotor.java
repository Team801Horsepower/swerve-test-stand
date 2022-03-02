package frc.robot.architecture;

/**
 * Defines the methods for motors which can rotate at a specific rate.
 */
public interface SpeedMotor {

    /**
     * Performs any required initialization. (ex. zero the encoder)
     */
    public default void init() {}

    /**
     * Performs any periodic tasks required by the motor. (ex. update PID loop)
     */
    public default void periodic() {}

    /**
     * Requests the output shaft rotate at the desired speed.
     * 
     * @param speed The speed to rotate in rad/s.
     */
    public void setDesiredSpeed(double speed);

    /**
     * Returns the current speed of the output shaft.
     * 
     * @return The speed the output shaft is running at in rad/s.
     * @apiNote This is not necessarily the last desired speed.
     */
    public double getCurrentSpeed();
}
