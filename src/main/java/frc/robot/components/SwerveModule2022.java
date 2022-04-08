package frc.robot.components;

import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule2022 extends SwerveModule {
    
    private static final double DRIVE_P = 0.005;
    private static final double DRIVE_I = 0.0;
    private static final double DRIVE_D = 0.0;

    private static final double DRIVE_GEAR_RATIO = 1.0 / 0.1875;
    private static final IdleMode DRIVE_IDLE_MODE = IdleMode.kCoast;

    private static final double TURN_P = 0.4;
    private static final double TURN_I = 0.000667;
    private static final double TURN_D = 0.1;
    private static final double TURN_I_MAX_ACCUM = 0.5;

    private static final double TURN_GEAR_RATIO = 10.0;
    
    private static final boolean TURN_INVERTED = false;

    private final Neo DRIVE_MOTOR;
    private final Neo550 TURN_MOTOR;
    private final ThroughboreEncoder THROUGHBORE_ENCODER;

    public SwerveModule2022(int driveCanId, int turnCanId, int throughborePin, boolean inverted) {
        super(
            new Neo(driveCanId),
            new Neo550(turnCanId),
            0.31918 / (2 * Math.PI)
        );

        // Set up the absolute encoder
        THROUGHBORE_ENCODER = new ThroughboreEncoder(throughborePin);

        // Set up the drive motor
        DRIVE_MOTOR = (Neo) super.DRIVE_MOTOR;
        int speedPid = DRIVE_MOTOR.getSpeedPid();
        DRIVE_MOTOR.PID.setP(DRIVE_P, speedPid);
        DRIVE_MOTOR.PID.setI(DRIVE_I, speedPid);
        DRIVE_MOTOR.PID.setD(DRIVE_D, speedPid);

        DRIVE_MOTOR.setGearRatio(DRIVE_GEAR_RATIO);

        DRIVE_MOTOR.CONTROLLER.setInverted(!inverted);
        DRIVE_MOTOR.CONTROLLER.setIdleMode(DRIVE_IDLE_MODE); // TODO: Switch back to kCoast
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
    public void init() {
        super.init();
        TURN_MOTOR.setPosition(THROUGHBORE_ENCODER.getCurrentPosition());
        resetZero();
    }

    @Override
    public void resetZero() {
        TURN_MOTOR.setPosition(0.0);
        THROUGHBORE_ENCODER.setPosition(0.0);
    }
    
}
