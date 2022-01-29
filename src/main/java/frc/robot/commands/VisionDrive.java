/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VisionDrive extends Command {

  private PhotonCamera camera;

  public VisionDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.s_stand);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    camera = new PhotonCamera("mmal_service_16.1");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.s_stand.update();
    //System.out.println("Running VisionDrive");
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      //System.out.println("hasTargets");
      System.out.println(-result.getBestTarget().getYaw());
      //REMOVE *4.5 after calibration
      Robot.s_stand.drive(-result.getBestTarget().getYaw()/180*Math.PI*4.5, 0.1);
    } else {
      System.out.println("!hasTargets");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

  }
}
