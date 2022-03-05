package frc.robot.components;

import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule2020 extends SwerveModule {

    private final Neo DRIVE_MOTOR;
    private final Neo550 TURN_MOTOR;

    public SwerveModule2020(int driveCanId, int turnCanId) {
        super(
            new Neo(driveCanId),
            new Neo550(turnCanId),
            0.31918 / (2 * Math.PI)
        );

        // Set up the drive motor
        DRIVE_MOTOR = (Neo) super.DRIVE_MOTOR;
        int speedPid = DRIVE_MOTOR.getSpeedPid();
        DRIVE_MOTOR.PID.setP(0.005, speedPid);
        DRIVE_MOTOR.PID.setI(0.0, speedPid);
        DRIVE_MOTOR.PID.setD(0.0, speedPid);
        DRIVE_MOTOR.PID.setFF(0.0, speedPid);
        DRIVE_MOTOR.PID.setIZone(0.0, speedPid);
        DRIVE_MOTOR.PID.setOutputRange(-1.0, 1.0);

        DRIVE_MOTOR.setGearRatio(0.1875);

        DRIVE_MOTOR.CONTROLLER.setInverted(true);
        DRIVE_MOTOR.CONTROLLER.setIdleMode(IdleMode.kCoast); // TODO: Switch back to kCoast
        DRIVE_MOTOR.CONTROLLER.setSmartCurrentLimit(40, 30);

        // Make settings persistent
        // DRIVE_MOTOR.CONTROLLER.burnFlash();

        // Set up the Turn Motor
        TURN_MOTOR = (Neo550) super.TURN_MOTOR;
        int positionPid = TURN_MOTOR.getPositionPid();
        TURN_MOTOR.PID.setP(0.5, positionPid);
        TURN_MOTOR.PID.setI(0.004, positionPid);
        TURN_MOTOR.PID.setD(0.7, positionPid);
        TURN_MOTOR.PID.setFF(0.0, positionPid);
        TURN_MOTOR.PID.setIMaxAccum(0.5, positionPid);
        TURN_MOTOR.PID.setOutputRange(-1.0, 1.0);
        
        TURN_MOTOR.setGearRatio(1.0 / 60.0);

        TURN_MOTOR.CONTROLLER.setInverted(true);
        TURN_MOTOR.CONTROLLER.setIdleMode(IdleMode.kCoast); // TODO: Switch back to kBrake
        TURN_MOTOR.CONTROLLER.setSmartCurrentLimit(30, 20);

        // Make settings persistent
        // TURN_MOTOR.CONTROLLER.burnFlash();
    }

    @Override
    public void resetZero() {
        TURN_MOTOR.setPosition(0.0);
    }

}
