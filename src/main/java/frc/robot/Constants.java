package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants {
        public static int NEO_ENCODER_CNTS_PER_REV = 42;

        public static double DRIVE_P = 0.0005;
        public static double DRIVE_I = 0.0;
        public static double DRIVE_D = 0.0;
        public static double DRIVE_IZ = 0.0;
        public static double DRIVE_FF = 0.000;
        public static double DRIVE_MAX_OUTPUT = 1.0;
        public static double DRIVE_MIN_OUTPUT = -1.0;
        public static boolean DRIVE_INVERT = true;
        public static IdleMode DRIVE_IDLEMODE = IdleMode.kCoast;

        public static int DRIVE_MAX_RPM = 18730;
        public static int DRIVE_MAX_CURRENT_STALL = 40;
        public static int DRIVE_MAX_CURRENT_RUN = 30;
        public static double MAX_ROBOT_SPEED = 1.0;
        public static double DRIVE_METERS_PER_RADIAN = 0.31918 * 0.1875 / (2 * Math.PI);

        public static double TURN_P = 0.05; // 0.5 gives a little overshoot on the test stand.
        public static double TURN_I = 0.0; // 0.004
        public static double TURN_D = 0.0; // 0.7
        public static double TURN_OUTPUT_LIMIT_LOW = -1;
        public static double TURN_OUTPUT_LIMIT_HIGH = 1;
        public static double TURN_MAX_I_OUT = 0.5;
        public static double TURN_OUTPUT_RAMPRATE = 1;
        public static double TURN_OUTPUT_FILTER = 0;
        public static double TURN_SETPOINT_RANGE = 2 * Math.PI;

        public static boolean TURN_INVERT = true;
        public static IdleMode TURN_IDLEMODE = IdleMode.kCoast;

        public static int TURN_MAX_CURRENT_STALL = 30;
        public static int TURN_MAX_CURRENT_RUN = 20;

        // Swerve Pod Motor CAN IDs
        public static int DRIVE_ID = 13; // Right Front
        public static int TURN_ID = 4; // 550 mini-NEO

}
