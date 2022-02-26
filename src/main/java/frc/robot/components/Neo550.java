package frc.robot.components;

public class Neo550 extends Neo {

    public static final String PART_NAME = "REV-21-1651";

    public static final double MAX_SPEED = 11000.0 / 60.0 * 2 * Math.PI;
    public static final double STALL_CURRENT = 100;
    public static final double STALL_TORQUE = 0.97;

    /**
     * Creates an instance of the Neo550 which refers to a Neo550 attached to a SparkMax.
     * Update the PID and motor configuration before use.
     * @param canId
     */
    public Neo550(int canId) {
        super(canId);
    } 

    @Override
    public void setDesiredSpeed(double speed) {
        setDesiredSpeed(speed, MAX_SPEED);
    }
}
