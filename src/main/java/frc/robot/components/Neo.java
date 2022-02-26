package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.architecture.PositionMotor;
import frc.robot.architecture.SpeedMotor;

public class Neo implements SpeedMotor, PositionMotor, Sendable {

    public static final String PART_NAME = "REV-21-1650";

    public static final double MAX_SPEED = 5650.0 / 60.0 * 2 * Math.PI;
    public static final double STALL_CURRENT = 105;
    public static final double STALL_TORQUE = 2.5;

    public final int CAN_ID;
    public final CANSparkMax CONTROLLER;
    public final RelativeEncoder ENCODER;
    public final SparkMaxPIDController PID;

    private int speedPid = 0, positionPid = 1;
    private int currentPid = 0;

    /**
     * Creates an instance of Neo which refers to a Neo or a Neo550 attached to a SparkMax.
     * Update the PID and motor configuration before use.
     * @param canId
     */
    public Neo(int canId) {
        CAN_ID = canId;
        CONTROLLER = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        ENCODER = CONTROLLER.getEncoder();
        ENCODER.setPositionConversionFactor(2 * Math.PI);
        ENCODER.setVelocityConversionFactor(2 * Math.PI / 60.0);
        PID = CONTROLLER.getPIDController();

        CONTROLLER.setSmartCurrentLimit(40);

        SendableRegistry.addLW(this, "Neo", canId);
    }

    @Override
    public void init() {
        // Zero the encoder on init instead of startup.
        ENCODER.setPosition(0.0);
    }

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Neo");
        builder.addBooleanProperty("inverted", () -> { return CONTROLLER.getInverted(); }, (value) -> { CONTROLLER.setInverted(value); });
        builder.addBooleanProperty("brake", () -> { return CONTROLLER.getIdleMode() == IdleMode.kBrake; }, (value) -> { CONTROLLER.setIdleMode(value ? IdleMode.kBrake : IdleMode.kCoast); });

        builder.addDoubleProperty("slot", () -> { return currentPid; }, (value) -> { currentPid = (int) value; });
        builder.addDoubleProperty("p", () -> { return PID.getP(currentPid); }, (value) -> { PID.setP(value, currentPid); });
        builder.addDoubleProperty("i", () -> { return PID.getI(currentPid); }, (value) -> { PID.setI(value, currentPid); });
        builder.addDoubleProperty("d", () -> { return PID.getD(currentPid); }, (value) -> { PID.setD(value, currentPid); });
        builder.addDoubleProperty("ff", () -> { return PID.getFF(currentPid); }, (value) -> { PID.setFF(value, currentPid); });
        builder.addDoubleProperty("iz", () -> { return PID.getIZone(currentPid); }, (value) -> { PID.setIZone(value, currentPid); });
    }

    protected void setDesiredSpeed(double speed, double maxSpeed) {
        if (speed > maxSpeed) {
            System.err.println("Tried to exceed max speed: " + speed + "r/s (max is "
                    + maxSpeed + "r/s)");
            speed = maxSpeed;
        }
        PID.setReference(speed, ControlType.kVelocity, speedPid);
    }
    
    @Override
    public void setDesiredSpeed(double speed) {
        setDesiredSpeed(speed, MAX_SPEED);
    }

    @Override
    public double getCurrentSpeed() {
        return ENCODER.getVelocity();
    }

    @Override
    public void setDesiredPosition(double position) {
        PID.setReference(position, ControlType.kPosition, positionPid);
    }

    @Override
    public double getCurrentPosition() {
        return ENCODER.getPosition();
    }

    public void setPositionPid(int pidSlot) {
        positionPid = pidSlot;
    }

    public int getPositionPid() {
        return positionPid;
    }

    public void setSpeedPid(int pidSlot) {
        speedPid = pidSlot;
    }

    public int getSpeedPid() {
        return speedPid;
    }
    
}
