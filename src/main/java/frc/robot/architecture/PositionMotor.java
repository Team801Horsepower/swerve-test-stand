package frc.robot.architecture;

import frc.robot.utilities.Utils;

/**
 * Defines the methods for motors which can turn to a specific position.
 */
public interface PositionMotor extends PositionEncoder {

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
     * @param position The position to turn to in radians.
     */
    public void setDesiredPosition(double position);

    /**
     * Requests the output shaft face the desired angle.
     * 
     * @param angle The angle to face in radians [0, 2PI).
     */
    public default void setDesiredAngle(double angle) {
        double currentHeading = Utils.normalizeAngle(getCurrentPosition());
        double desiredHeading = Utils.normalizeAngle(angle);
        double desiredPosition = getCurrentPosition();
        if (Math.abs(currentHeading - desiredHeading) < Math.PI) {
            desiredPosition += desiredHeading - currentHeading;
        } else if (currentHeading > Math.PI) {
            desiredPosition += (2 * Math.PI) + desiredHeading - currentHeading;
        } else {
            desiredPosition += -(2 * Math.PI) + desiredHeading - currentHeading;
        }
        
        setDesiredPosition(desiredPosition);  
    };
}

