package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public class Constants
{
    public static final double DRIVE_P = 0.0005;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_IZ = 0.0;
    public static final double DRIVE_FF = 0.000;
    public static final double DRIVE_MAX_OUTPUT = 1.0;
    public static final double DRIVE_MIN_OUTPUT = -1.0;
    public static final boolean DRIVE_INVERT[] = {false, false, false, true}; //TODO: fix for later motor values
    public static final IdleMode DRIVE_IDLEMODE[] = {IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast}; 

    public static final int DRIVE_MAX_RPM = 18730;
    public static final int DRIVE_MAX_CURRENT_STALL = 40;
    public static final int DRIVE_MAX_CURRENT_RUN = 30;



    public static final double TURN_P = 0.5;  // 0.5 gives a little overshoot on the test stand.
    public static final double TURN_I = 0.004;  // 0.004
    public static final double TURN_D = 0.7; // 0.7
    public static final double OutputLowLimit = -1;
    public static final double OutputHighLimit = 1;
    public static final double MaxIOutput = 0.5;
    public static final double OutputRampRate = .1;
    public static final double OutputFilter = 0;
    public static final double SetpointRange = 2 * Math.PI;

    public static final boolean TURN_INVERT[] = {false, false, false, false}; //TODO: fix for later motor values
    public static final IdleMode TURN_IDLEMODE[] = {IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake};

    public static final int TURN_MAX_CURRENT_STALL = 30;
    public static final int TURN_MAX_CURRENT_RUN = 10;

    public static final int DRIVE_POD_ID = 13;
    public static final int TURN_POD_ID = 4;

}