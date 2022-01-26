package frc.robot.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class DriveMotor {
    private double currentRPM = 0.0;
    private double desiredRPM = 0.0;

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

        sparkMotor.setInverted(Constants.DRIVE_INVERT[motorIndex]);
        sparkMotor.setIdleMode(Constants.DRIVE_IDLEMODE[motorIndex]);
        sparkMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL,
                Constants.DRIVE_MAX_CURRENT_RUN);
    }

    /**
     * 
     * @param rpm desired RPMs of the motor shaft
     */
    public void setDesiredRPM(double rpm) {
        desiredRPM = rpm;
        sparkPID.setReference(desiredRPM, CANSparkMax.ControlType.kDutyCycle);
    }

    /**
     * 
     * @return motor shaft velocity in RPM
     */
    public double getCurrentRPM() {
        currentRPM = sparkEncoder.getVelocity();
        return currentRPM;
    }
}
