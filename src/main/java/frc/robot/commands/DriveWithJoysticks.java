/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.IO;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class DriveWithJoysticks extends CommandBase {
    
    double lastAngular = 0.0;
    boolean fieldOrient = true;

    /**
     * Creates a new RobotDriveWithJoysticks.
     */
    public DriveWithJoysticks(boolean fieldOrient) {
        addRequirements(RobotContainer.CHASSIS);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        double forward = IO.Joystick.DriverLeft.getForward();
        double leftward = IO.Joystick.DriverLeft.getLeftward();
        double angular = IO.Joystick.DriverRight.getLeftward();
        
        forward *= Chassis.MAX_DRIVE_SPEED;
        leftward *= Chassis.MAX_DRIVE_SPEED;
        angular *= Chassis.MAX_TURN_SPEED;
        
        if (lastAngular != 0.0 && angular == 0.0) {
            RobotContainer.CHASSIS.setHeading(RobotContainer.CHASSIS.getCurrentPose().getRotation().getRadians(), true);
        }

        lastAngular = angular;
        if (fieldOrient) {
            RobotContainer.CHASSIS.fieldDrive(forward, leftward, angular);
        } else {
            RobotContainer.CHASSIS.robotDrive(forward, leftward, angular);
        }
    }

    public void toggleMode() {
        fieldOrient = !fieldOrient;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}