/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.VisionDrive;
import frc.robot.commands.JoystickDrive;
import frc.robot.components.SwervePod;
import frc.robot.utilities.Utils;

/**
 * Add your docs here.
 */
public class Stand extends Subsystem {

    private SwervePod pod = new SwervePod(Constants.DRIVE_POD_ID, Constants.TURN_POD_ID, 0);

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new VisionDrive());
    }

    public void update() {
        pod.processPod();
    }

    public void joystickDrive() {
        update();

        //double x = Robot.m_oi.getDriverX();
        double x = Robot.io.getDriverLeftX();
        double y = Robot.io.getDriverLeftY();

        double speed = Utils.magnitude(x, y);


        // double theta = Utils.angle(Robot.io.getDriverLeftX(), Robot.io.getDriverLeftY());

        //double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) );

        //pod.setDesiredRPM(Utils.map(y, -1, 1, -8, 8));
        //System.out.println(theta);
        pod.setDesiredRPM(speed);

        double angle = Math.atan2(y, x);

        pod.setDesiredAngle(angle);
    }

    public void drive(double angle, double speed) {
        pod.setDesiredRPM(speed);
        pod.setDesiredAngle(angle);
    }

    public double getAngle() {
        return pod.getCurrentAngle();
    }
}
