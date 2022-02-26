// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.architecture;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Provides a base from which all {@code Drive} systems must inherit. */
public abstract class Drive {

    /**
     * Requests the {@code Drive} to move at the desired speeds.
     * 
     * @param vx The velocity (m/s) in the forwards direction.
     * @param vy The velocity (m/s) in the left direction.
     * @param omega The angular velocity (rad/s) in the counterclockwise direction.
     */
    public void setDesiredSpeeds(double vx, double vy, double omega) {
        setDesiredSpeeds(new ChassisSpeeds(vx, vy, omega));
    }

    /**
     * Performs any required initialization. (ex. call motor {@code init() functions})
     */
    public void init() {}

    /**
     * Performs any periodic tasks required by the {@code Drive} system. (ex. update PID loop)
     */
    public void periodic() {}

    /**
     * Request the {@code Drive} to move at the desired speeds.
     * 
     * @param speeds The speeds as defined by wpilib.
     */
    public abstract void setDesiredSpeeds(ChassisSpeeds speeds);

    /**
     * Returns the current speeds of the {@code Drive} system.
     *
     * @return The speeds as defined by wpilib.
     * @apiNote This is not necessarily the last desired speeds.
     */
    public abstract ChassisSpeeds getCurrentSpeeds();

    /**
     * Resets the current pose estimation to match the given pose.
     * 
     * @param newPose A more accurate pose estimate (hopefully).
     */
    public abstract void resetPose(Pose2d newPose);

    /**
     * Returns the most recent pose estimate from odometry.
     * 
     * @return The current pose estimate from odometry.
     */
    public abstract Pose2d getCurrentPose();
}
