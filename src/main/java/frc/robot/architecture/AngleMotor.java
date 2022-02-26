package frc.robot.architecture;

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
            double currentHeading = positionMotor.getCurrentPosition() % 1;
            double desiredHeading = angle / (2 * Math.PI);
            double desiredPosition = positionMotor.getCurrentPosition();
            if (Math.abs(currentHeading - desiredHeading) < 0.5) {
                desiredPosition += desiredHeading - currentHeading;
            } else if (currentHeading > 0.5) {
                desiredPosition += 1 + desiredHeading - currentHeading;
            } else {
                desiredHeading += -1 + desiredHeading - currentHeading;
            }
            positionMotor.setDesiredPosition(desiredPosition);                
        }

        @Override
        public double getCurrentAngle() {
            return (positionMotor.getCurrentPosition() * 2 * Math.PI) % (2 * Math.PI);
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
     * Requests the motor face the desired angle.
     * 
     * @param angle The angle to face in radians.
     */
    public void setDesiredAngle(double angle);

    /**
     * Returns the current heading of the motor.
     * 
     * @return The current angle the motor is facing.
     * @apiNote This is not necessarily the last desired angle.
     */
    public double getCurrentAngle();

    public static WrappedPositionMotor fromPositionMotor(PositionMotor positionMotor) {
        return new WrappedPositionMotor(positionMotor);
    }
}
