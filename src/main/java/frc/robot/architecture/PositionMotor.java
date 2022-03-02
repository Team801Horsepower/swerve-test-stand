package frc.robot.architecture;

/**
 * Defines the methods for motors which can turn to a specific position.
 */
public interface PositionMotor {

    /**
     * Performs any required initialization. (ex. zero the encoder)
     */
    public default void init() {};

    /**
     * Performs any periodic tasks required by the motor. (ex. update PID loop)
     */
    public default void periodic() {}

    /**
     * Requests the output shaft turn to the desired position.
     * 
     * @param position The output shaft to turn to in radians.
     */
    public void setDesiredPosition(double position);

    /**
     * Returns the current position of the output shaft.
     * 
     * @return The current position of the output shaft in radians.
     * @apiNote This is not necessarily the last desired position.
     */
    public double getCurrentPosition();
}

