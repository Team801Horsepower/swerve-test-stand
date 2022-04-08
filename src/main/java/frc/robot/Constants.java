package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Aim;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.Gather;
import frc.robot.commands.PathPlannerControllerCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Chassis;
import frc.robot.utilities.Utils;

import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    // SPEEDS
    public static final double CLIMB_SPEED = 0.4;

    // SETPOINTS
    public static final double CLIMB_PRIME_POSITION = -1.00;
    public static final double CLAW_HIGH_POSITION = 2.369;
    public static final double CLAW_RELEASE_HIGH_POSITION = -0.343;
    public static final double CLAW_TRAVERSAL_POSITION = -2.4;

    public static final double ARM_LOWERED_POSITION = -1.0;
    public static final double ARM_RAISED_POSITION = 0.0;

    public static final double FLYWHEEL_FREE_BALL_ROTATION = Units.degreesToRadians(-270.0);
    public static final double GATHER_FIRE_ROTATION = Math.PI;
    public static final double GATHER_TAMP_ROTATION = -2.0 * Math.PI;

    public static final double FEEDER_1_BALL = 6.0 * Math.PI;
    public static final double FEEDER_TAMP_BALL = -2.0 * Math.PI;
    public static final int FEEDER_THRESHOLD = 800;

    // HARDWARE CONFIGURATIONS

    // Climber CAN IDs
    public static final int CLIMB_RIGHT = 4;
    public static final int CLIMB_RIGHT_CLAW = 5;
    public static final int CLIMB_LEFT = 17;
    public static final int CLIMB_LEFT_CLAW = 16;

    // Gather Constants
    public static final int GATHER_WHEELS = 15;
    public static final int GATHER_ARM = 14;

    // Shooter CAN IDs
    public static final int SHOOTER = 7;
    public static final int FEEDER = 5;

    // FIELD PARAMETERS
    public static final Pose2d GOAL_POSE = new Pose2d(new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(0.5 * 27.0)), new Rotation2d(Units.degreesToRadians(24.0)));
    public static final Translation2d TARGET_LOCK_POSITION = new Translation2d(-0.1, 4.0);
    public static final double TARGET_HEIGHT = Units.inchesToMeters(8.0 * 12.0 + 7.0 - 2.0);
    public static final double TARGET_TAPE_WIDTH = Units.inchesToMeters(2.0);
    public static final double TARGET_RADIUS = Units.inchesToMeters(26.0 + 11.0 / 16.0);
    public static final double TARGET_OFFSET_ANGLE = Units.degreesToRadians(9.75);

    // AUTO PATHS
    public static final Map<String, PathPlannerTrajectory> AUTO_PATHS = Map.of(
        "Drive Backwards", PathPlanner.loadPath("Drive Backwards", Chassis.MAX_DRIVE_SPEED, Chassis.MAX_DRIVE_ACCELERATION),
        "Bottom 2 Ball", PathPlanner.loadPath("Bottom 2 Ball", Chassis.MAX_DRIVE_SPEED, Chassis.MAX_DRIVE_ACCELERATION)
        
    );
    
    public static final AutonomousRoutine[] AUTO_ROUTINES = {
        new AutonomousRoutine(
                new DriveToPose(new Pose2d(4.39, 4.09, new Rotation2d(0.0)), 0.1).andThen(
                new Aim(),
                new Shoot().deadlineWith(new Aim())),
            new Pose2d(6.09, 4.09, new Rotation2d(0.0))
        ),
        new AutonomousRoutine(
                new DriveToPose(new Pose2d(5.06, 6.16, Rotation2d.fromDegrees(129.19)), 0.1).alongWith(new Gather(false)).andThen(
                    RobotContainer.CHASSIS.headingCommand(Units.degreesToRadians(340.0)),
                    new Aim(),
                    new Shoot().deadlineWith(new Aim())),
            new Pose2d(7.01, 4.74, Rotation2d.fromDegrees(159.04))
        ),
        new AutonomousRoutine(
                new DriveToPose(new Pose2d(5.20, 1.92, Rotation2d.fromDegrees(-163.93 + 360.0)), 0.1).alongWith(new Gather(false)).andThen(
                    RobotContainer.CHASSIS.headingCommand(Units.degreesToRadians(40.0)),
                    new Aim(),
                    new Shoot().deadlineWith(new Aim())),
            new Pose2d(7.65, 2.87, Rotation2d.fromDegrees(-110.56 + 360.0))
        ),
    };

    public static class AutonomousRoutine {

        public final Pose2d INITIAL_POSE;
        public final Command COMMAND;

        private AutonomousRoutine(Command command, Pose2d initialPose) {
            this.COMMAND = command;
            this.INITIAL_POSE = initialPose;
        }
    }
}
