package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Reset extends CommandBase {

    public Reset()
    {
        addRequirements(RobotContainer.chassis);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println("Reset");
        RobotContainer.chassis.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
