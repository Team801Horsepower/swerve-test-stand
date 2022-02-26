package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import frc.robot.architecture.SpeedMotor;

public class DriveMotor implements SpeedMotor {
    public static final double MAX_RPM = 5650.0;

    private double currentSpeed = 0.0;
    private double desiredSpeed = 0.0;

    private SparkMaxPIDController sparkPID;
    private CANSparkMax sparkMotor;
    private RelativeEncoder sparkEncoder;

    /**
     *
     * @param motorID of the Spark Max
     * @param motorIndex of the motor index
     */
    public DriveMotor(int motorID, int motorIndex) {
        sparkMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        sparkPID = sparkMotor.getPIDController();
        sparkEncoder = sparkMotor.getEncoder();

        sparkPID.setP(Constants.DRIVE_P);
        sparkPID.setI(Constants.DRIVE_I);
        sparkPID.setD(Constants.DRIVE_D);
        sparkPID.setIZone(Constants.DRIVE_IZ);
        sparkPID.setFF(Constants.DRIVE_FF);
        sparkPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

        sparkMotor.setInverted(Constants.DRIVE_INVERT);
        sparkMotor.setIdleMode(Constants.DRIVE_IDLEMODE);
        sparkMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL,
                Constants.DRIVE_MAX_CURRENT_RUN);
    }

    @Override
    public void setDesiredSpeed(double speed) {
        desiredSpeed = speed / (2 * Math.PI) * 60.0;
        if (desiredSpeed > MAX_RPM) {
            System.err.println("Tried to exceed max speed: " + desiredSpeed + " rad/s (max is "
                    + MAX_RPM + " rad/s)");
            desiredSpeed = MAX_RPM;
        }
        sparkPID.setReference(desiredSpeed, ControlType.kVelocity);
    }

    @Override
    public double getCurrentSpeed() {
        currentSpeed = sparkEncoder.getVelocity() * 2 * Math.PI / 60;
        return currentSpeed;
    }
}
