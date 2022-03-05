package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModule2020;
import frc.robot.components.SwerveModule2022;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase {

    // Front
    // 2 1
    // 3 4

    SwerveModule module2020;
    SwerveModule module2022;

    public Chassis() {
        super();

        module2020 = new SwerveModule2020(Constants.DRIVE_ID, Constants.TURN_ID);;
        module2022 = new SwerveModule2022(15, 14, 0);
    }

    /**
     * This method must be called before the Chassis is used
     */
    public void init() {
        module2020.init();
        module2022.init();
    }

    public void periodic() {
        module2020.periodic();
        module2022.periodic();
    }

    /**
     * This method will be called once per scheduler run in Autonomous
     */

    public void directionDrive(double speed, double angle) {
        module2020.setDesiredAngle(angle);
        module2020.setDesiredSpeed(speed);
        module2022.setDesiredAngle(angle);
        module2022.setDesiredSpeed(speed);
    }

    public void stop() {
        module2020.setDesiredState(new SwerveModuleState());
        module2022.setDesiredState(new SwerveModuleState());
    }

    public void reset() {
        System.out.println("Resetting");
        module2020.resetZero();
        module2022.resetZero();
    }
}
