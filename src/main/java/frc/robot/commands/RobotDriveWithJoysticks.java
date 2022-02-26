/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RobotDriveWithJoysticks extends CommandBase {
    /**
     * Creates a new RobotDriveWithJoysticks.
     */
    public RobotDriveWithJoysticks() {
        addRequirements(RobotContainer.chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = RobotContainer.io.getDriverLeftX();
        double y = RobotContainer.io.getDriverLeftY();

        double mag = x * x + y * y;
        if (mag > 1) {
            x /= mag;
            y /= mag;
            mag = 1;
        }
        double angle = Math.atan2(y, x);

        RobotContainer.chassis.directionDrive(mag, angle);
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
