package frc.robot.components;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.architecture.AngleMotor;
import frc.robot.architecture.AngleMotor.WrappedPositionMotor;

public class SwerveModule2022 extends SwerveModule {

    private final Neo DRIVE_MOTOR;
    private final Neo550 TURN_MOTOR;
    private final DutyCycle THROUGHBORE_DUTYCYCLE;

    public SwerveModule2022(int driveCanId, int turnCanId, int throughborePin) {
        super(
            new Neo(driveCanId),
            AngleMotor.fromPositionMotor(new Neo550(turnCanId)),
            0.31918 / (2 * Math.PI)
        );

        // Set up the absolute encoder
        THROUGHBORE_DUTYCYCLE = new DutyCycle(new DigitalInput(throughborePin));

        // Set up the drive motor
        DRIVE_MOTOR = (Neo) super.DRIVE_MOTOR;
        int speedPid = DRIVE_MOTOR.getSpeedPid();
        DRIVE_MOTOR.PID.setP(0.0005, speedPid);
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
        DRIVE_MOTOR.CONTROLLER.burnFlash();

        // Set up the Turn Motor
        TURN_MOTOR = (Neo550) ((WrappedPositionMotor) super.TURN_MOTOR).positionMotor;
        int positionPid = TURN_MOTOR.getPositionPid();
        TURN_MOTOR.PID.setP(0.08333, positionPid);
        TURN_MOTOR.PID.setI(0.000667, positionPid);
        TURN_MOTOR.PID.setD(0.1, positionPid);
        TURN_MOTOR.PID.setFF(0.0, positionPid);
        TURN_MOTOR.PID.setIMaxAccum(0.5, positionPid);
        TURN_MOTOR.PID.setOutputRange(-1.0, 1.0);

        TURN_MOTOR.setGearRatio(1 / 10);
        
        TURN_MOTOR.CONTROLLER.setInverted(true);
        TURN_MOTOR.CONTROLLER.setIdleMode(IdleMode.kCoast); // TODO: Switch back to kBrake
        TURN_MOTOR.CONTROLLER.setSmartCurrentLimit(30, 20);

        // Make settings persistent
        TURN_MOTOR.CONTROLLER.burnFlash();
    }

    @Override
    public void init() {
        super.init();
        TURN_MOTOR.ENCODER.setPosition(THROUGHBORE_DUTYCYCLE.getOutput() * 2 * Math.PI);
    }
    
}
