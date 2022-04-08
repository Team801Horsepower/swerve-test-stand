/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.Chassis;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    @SuppressWarnings("unused")
    private static final IO _IO = new IO();

    public static final Chassis CHASSIS = new Chassis();

    public static final PowerDistribution POWER_DISTRIBUTION = new PowerDistribution();
    public static final Constants.AutonomousRoutine AUTONOMOUS_ROUTINE = Constants.AUTO_ROUTINES[Preferences.getInt("AUTO_ROUTINE", 0)];

    private final DriveWithJoysticks DRIVE_COMMAND = new DriveWithJoysticks(true);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CHASSIS.setDefaultCommand(DRIVE_COMMAND);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AUTONOMOUS_ROUTINE.COMMAND;
    }
}
