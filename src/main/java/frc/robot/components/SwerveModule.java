package frc.robot.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.architecture.PositionMotor;
import frc.robot.architecture.SpeedMotor;

/** A class which implements the boilerplate code for running a typical swerve module. */
public abstract class SwerveModule {

    protected final double METERS_PER_RAD;

    protected final SpeedMotor DRIVE_MOTOR;
    protected final PositionMotor TURN_MOTOR;

    private double lastAngle;
    private boolean flipFlag = false;

    private SwerveModuleState state;

    /**
     * Creates a new `SwerveModule` instance
     * 
     * @param driveMotor A motor which implements the `SpeedMotor` interface.
     * @param turnMotor A motor which implements the `TurnMotor` interface.
     * @param metersPerRad A conversion constant with the units meters/rad which converts from
     *        radians to meters. (Depends wheel size)
     */
    public SwerveModule(SpeedMotor driveMotor, PositionMotor turnMotor, double metersPerRad) {
        this.DRIVE_MOTOR = driveMotor;
        this.TURN_MOTOR = turnMotor;
        this.METERS_PER_RAD = metersPerRad;
        this.state = new SwerveModuleState();
    }

    /** Initializes both motors. */
    public void init() {
        DRIVE_MOTOR.init();
        TURN_MOTOR.init();
    }

    /** Calls `.periodic()` on both motors. */
    public void periodic() {
        DRIVE_MOTOR.periodic();
        TURN_MOTOR.periodic();
    }

    /**
     * Convenience function to set Speed and Angle at the same time.
     * 
     * @param state A `SwerveModuleState` describing the desired state.
     */
    public void setDesiredState(SwerveModuleState state) {
        setDesiredAngle(state.angle.getRadians());
        setDesiredSpeed(state.speedMetersPerSecond);
    }

    /**
     * Requests the module to drive at the desired speed.
     * 
     * @param speed The speed to rotate in m/s.
     */
    public void setDesiredSpeed(double speed) {
        if (flipFlag) {
            speed = -speed;
        }
        DRIVE_MOTOR.setDesiredSpeed(speed / METERS_PER_RAD);
    }

    /**
     * Requests the module face the desired angle.
     * 
     * @param angle The angle to face in radians.
     */
    public void setDesiredAngle(double angle) {
        angle %= 2 * Math.PI;
        double errorAngle = Math.abs(angle - lastAngle);
        lastAngle = angle;

        if (errorAngle > Math.PI / 2 && errorAngle < 3 * Math.PI / 2) {
            // TODO: Re-enable flipFlag
            flipFlag = !flipFlag;
        }

        if (flipFlag) {
            angle = (angle + Math.PI) % (2 * Math.PI);
        }

        TURN_MOTOR.setDesiredAngle(angle);
    }

    /**
     * Returns the current Speed and Angle in a `SwerveModuleState`.
     */
    public SwerveModuleState getCurrentState() {
        state.angle = new Rotation2d(TURN_MOTOR.getCurrentAngle());
        state.speedMetersPerSecond = DRIVE_MOTOR.getCurrentSpeed() * METERS_PER_RAD;
        return state;
    }

    public abstract void resetZero();
}
