package frc.robot.architecture;

import frc.robot.utilities.Utils;

/**
 * Defines the methods for motors which can turn to face a specific angle.
 */
public interface AngleMotor {

    public class WrappedPositionMotor implements AngleMotor {
        public final PositionMotor positionMotor;

        private WrappedPositionMotor(PositionMotor positionMotor) {
            this.positionMotor = positionMotor;
        }
            
        @Override
        public void setDesiredAngle(double angle) {
            double currentHeading = Utils.normalizeAngle(positionMotor.getCurrentPosition());
            double desiredHeading = Utils.normalizeAngle(angle);
            double desiredPosition = positionMotor.getCurrentPosition();
            if (Math.abs(currentHeading - desiredHeading) < Math.PI) {
                desiredPosition += desiredHeading - currentHeading;
            } else if (currentHeading > Math.PI) {
                desiredPosition += (2 * Math.PI) + desiredHeading - currentHeading;
            } else {
                desiredPosition += -(2 * Math.PI) + desiredHeading - currentHeading;
            }
            System.out.println("HEADING: " + desiredHeading);
            System.out.println("POSITION: " + desiredPosition);
            positionMotor.setDesiredPosition(desiredPosition);                
        }

        @Override
        public double getCurrentAngle() {
            return positionMotor.getCurrentPosition() % (2 * Math.PI);
        }
    }

    /**
     * Performs any required initialization. (ex. zero the encoder)
     */
    public default void init() {};

    /**
     * Performs any periodic tasks required by the motor. (ex. update PID loop)
     */
    public default void periodic() {}

    /**
     * Requests the output shaft face the desired angle.
     * 
     * @param angle The angle to face in radians.
     */
    public void setDesiredAngle(double angle);

    /**
     * Returns the current heading of the output shaft.
     * 
     * @return The current angle the output shaft is facing.
     * @apiNote This is not necessarily the last desired angle.
     */
    public double getCurrentAngle();

    public static WrappedPositionMotor fromPositionMotor(PositionMotor positionMotor) {
        return new WrappedPositionMotor(positionMotor);
    }
}
