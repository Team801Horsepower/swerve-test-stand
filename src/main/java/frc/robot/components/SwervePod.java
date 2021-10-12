package frc.robot.components;

public class SwervePod
{

    private DriveMotor driveMotor;
    private TurnMotor turnMotor;
    private double lastDesiredAngle;

    // In order to stear the least,
    private boolean flipFlag = false;   // keeps track of the current 'forwad' direction

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
        if (flipFlag)
        {
            driveMotor.setDesiredRPM(-speed);
        }
        else
        {
            driveMotor.setDesiredRPM(speed);
        }
    }

    public void setDesiredAngle(double desiredAngle)
    {
        // Check to see if the new desired angle is more than 90 degrees away from the last
        // desired angle.  If it is, toggle the flip flag.  The flip flag swaps the drive motor
        // directing and rotates the steer input angle by 180 degrees.

        double errorAngle = Math.abs(desiredAngle - lastDesiredAngle);
        lastDesiredAngle = desiredAngle;

        // toggle the flip flag if angle is 90 out.
        if (errorAngle > Math.PI/2 && errorAngle < 3 * Math.PI/2 )
        {
            if (flipFlag)
            {
                flipFlag = false;
            }
            else
            {
                flipFlag = true;
            }
        }

        if (flipFlag)
        {
            double tempAngle = ( desiredAngle + Math.PI ) % (2 * Math.PI);
            turnMotor.setDesiredAngle(tempAngle);
        }
        else
        {
            turnMotor.setDesiredAngle(desiredAngle);
        }
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
