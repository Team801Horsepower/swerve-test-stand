package frc.robot.components;

import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule2020 extends SwerveModule {

    private static final double DRIVE_P = 0.030;
    private static final double DRIVE_I = 0.0;
    private static final double DRIVE_D = 0.0;
    private static final double DRIVE_FF = 0.005;

    private static final double DRIVE_GEAR_RATIO = 1.0 / 0.1875;
    private static final IdleMode DRIVE_IDLE_MODE = IdleMode.kCoast;

    private static final double TURN_P = 0.5;
    private static final double TURN_I = 0.004;
    private static final double TURN_D = 0.7;
    private static final double TURN_I_MAX_ACCUM = 0.5;

    private static final double TURN_GEAR_RATIO = 60.0;
    private static final boolean TURN_INVERTED = false;

    private final Neo DRIVE_MOTOR;
    private final Neo550 TURN_MOTOR;

    public SwerveModule2020(int driveCanId, int turnCanId, boolean inverted) {
        super(
            new Neo(driveCanId),
            new Neo550(turnCanId),
            0.31918 / (2 * Math.PI)
        );

        // Set up the drive motor
        DRIVE_MOTOR = (Neo) super.DRIVE_MOTOR;
        int speedPid = DRIVE_MOTOR.getSpeedPid();
        DRIVE_MOTOR.PID.setP(DRIVE_P, speedPid);
        DRIVE_MOTOR.PID.setI(DRIVE_I, speedPid);
        DRIVE_MOTOR.PID.setD(DRIVE_D, speedPid);
        DRIVE_MOTOR.PID.setFF(DRIVE_FF, speedPid);
        DRIVE_MOTOR.PID.setIZone(0.0, speedPid);
        DRIVE_MOTOR.PID.setOutputRange(-1.0, 1.0);

        DRIVE_MOTOR.setGearRatio(DRIVE_GEAR_RATIO);

        DRIVE_MOTOR.CONTROLLER.setInverted(!inverted);
        DRIVE_MOTOR.CONTROLLER.setIdleMode(DRIVE_IDLE_MODE);
        DRIVE_MOTOR.CONTROLLER.setSmartCurrentLimit(40, 30);

        // Make settings persistent
        // DRIVE_MOTOR.CONTROLLER.burnFlash();

        // Set up the Turn Motor
        TURN_MOTOR = (Neo550) super.TURN_MOTOR;
        int positionPid = TURN_MOTOR.getPositionPid();
        TURN_MOTOR.PID.setP(TURN_P, positionPid);
        TURN_MOTOR.PID.setI(TURN_I, positionPid);
        TURN_MOTOR.PID.setD(TURN_D, positionPid);
        TURN_MOTOR.PID.setIMaxAccum(TURN_I_MAX_ACCUM, positionPid);
        
        TURN_MOTOR.setGearRatio(TURN_GEAR_RATIO);

        TURN_MOTOR.CONTROLLER.setInverted(TURN_INVERTED);
        TURN_MOTOR.CONTROLLER.setSmartCurrentLimit(30, 20);

        // Make settings persistent
        // TURN_MOTOR.CONTROLLER.burnFlash();
    }

    @Override
    public void resetZero() {
        TURN_MOTOR.setPosition(0.0);
    }

}
