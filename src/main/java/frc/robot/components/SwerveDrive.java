package frc.robot.components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.architecture.Drive;
import frc.robot.architecture.PositionEncoder;

/**
 * Implements a standard Swerve Drive system.
 */
public class SwerveDrive extends Drive {

    private final SwerveDriveKinematics kinematics;
    private final PositionEncoder gyro;
    private final SwerveDriveOdometry odometry;
    private final SwerveModule[] modules;

    private Pose2d odometricPose;


    /**
     * Creates a new instance of {@code SwerveDrive}.
     * 
     * @param modules An array of {@code SwerveModules}.
     * @param offsets An array describing the offset of each {@code SwerveModule} from the center of
     *        the robot.
     * @param gyro A reference to a {@code Gyro} to use during odometry.
     */
    public SwerveDrive(SwerveModule[] modules, Translation2d[] offsets, PositionEncoder gyro) {
        this(modules, offsets, gyro, new Pose2d());
    }

    /**
     * Creates a new instance of {@code SwerveDrive}.
     * 
     * @param modules An array of {@code SwerveModules}.
     * @param offsets An array describing the offset of each {@code SwerveModule} from the center of
     *        the robot.
     * @param gyro A reference to a {@code Gyro} to use during odometry.
     * @param initalPose A {@code Pose2d} describing the starting configuration of the robot.
     */
    public SwerveDrive(SwerveModule[] modules, Translation2d[] offsets, PositionEncoder gyro,
            Pose2d initialPose) {
        assert modules.length == offsets.length;

        this.kinematics = new SwerveDriveKinematics(offsets);
        this.modules = modules;

        this.gyro = gyro;
        this.odometricPose = initialPose;
        this.odometry = new SwerveDriveOdometry(kinematics,
                new Rotation2d(gyro.getCurrentAngle()), initialPose);
    }

    @Override
    public void init() {
        for (SwerveModule module : modules) {
            module.init();
        }
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }
        odometricPose =
                odometry.update(new Rotation2d(gyro.getCurrentAngle()), getCurrentModuleStates());
    }

    @Override
    public void setDesiredSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    @Override
    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }

    private SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = modules[i].getCurrentState();
        }
        return moduleStates;
    }

    @Override
    public Pose2d getCurrentPose() {
        return odometricPose;
    }

    @Override
    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(newPose, new Rotation2d(gyro.getCurrentAngle()));
    }

    @Override
    public void reset() {
        for (SwerveModule module : modules) {
            module.resetZero();
        }
    }
}
