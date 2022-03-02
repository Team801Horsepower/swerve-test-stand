package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModule2020;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase {

    // Front
    // 2 1
    // 3 4

    SwerveModule module;

    public Chassis() {
        super();

        module = new SwerveModule2020(Constants.DRIVE_ID, Constants.TURN_ID);;
    }

    /**
     * This method must be called before the Chassis is used
     */
    public void init() {
        module.init();
    }

    public void periodic() {
        module.periodic();
    }

    /**
     * This method will be called once per scheduler run in Autonomous
     */

    public void directionDrive(double speed, double angle) {
        module.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angle)));
    }

    public void stop() {
        module.setDesiredState(new SwerveModuleState());
    }
}
