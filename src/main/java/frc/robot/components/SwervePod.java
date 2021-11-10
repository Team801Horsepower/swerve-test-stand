package frc.robot.components;

public class SwervePod
{

    private DriveMotor driveMotor;
    private TurnMotor turnMotor;
    private double lastDesiredAngle;

    public SwervePod(int driveMotorID, int turnMotorID, int podIndex)
    {
        driveMotor = new DriveMotor(driveMotorID, podIndex);
        turnMotor = new TurnMotor(turnMotorID, podIndex);
    }

    // main process to keep updating values and PID processing
    public void processPod()
    {
        // driveMotor.processDrive();  // Not currently needed because Sparc does PID work
        turnMotor.processTurn();
    }

    public void setDesiredRPM(double speed)
    {
        driveMotor.setDesiredRPM(speed);
    }

    public void setDesiredAngle(double desiredAngle)
    {
        // Check to see if the new desired angle is more than 90 degrees away from the last
        // desired angle.  If it is, toggle the flip flag.  The flip flag swaps the drive motor
        // directing and rotates the steer input angle by 180 degrees.

        double errorAngle = Math.abs(desiredAngle - lastDesiredAngle);
        lastDesiredAngle = desiredAngle;

        turnMotor.setDesiredAngle(desiredAngle);
    }

    public double getCurrentRPM()
    {
        return driveMotor.getCurrentRPM();
    }

    public double getCurrentAngle()
    {
        return turnMotor.getCurrentAngle();
    }

    public void zeroEncoder()
    {
      turnMotor.zeroEncoder();
    }

}
