package frc.robot.components;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.CAN;
import frc.robot.Constants;
import frc.robot.utilities.*;

public class TurnMotor
{
  // heading about a unit circle in radians.
  private double currentAngle = 0.0;  // rotates about the Z axis [0,2PI) rad.
  private double desiredAngle = 0.0;  // rotates about the Z axis [0,2PI) rad.

  //private CANPIDController sparkPID;
  private CANSparkMax sparkMotor;
  private CANEncoder sparkEncoder;

  private PID anglePID = null;

  // error signal to motor range -1 to 1 based PID error with angle inputs ranging [0 to 2PI)
  private static double vTheta;

  public TurnMotor(int motorID, int motorIndex)
  {
      sparkMotor = new CANSparkMax(motorID, MotorType.kBrushless);

      sparkEncoder = sparkMotor.getEncoder(EncoderType.kHallSensor, 42);
      // sparkEncoder = sparkMotor.getEncoder(EncoderType.kQuadrature, 4096 * 6);
      // sparkEncoder = sparkMotor.getEncoder(EncoderType.kQuadrature, 8192 * 6);

      sparkEncoder.setPositionConversionFactor(2*Math.PI / 60);  // encoder will return radians
      // zero the encoder on init to avoid haveing to power off the bot every time.
      sparkEncoder.setPosition(0.0);

      sparkMotor.setInverted(Constants.TURN_INVERT[motorIndex]);
      sparkMotor.setIdleMode(Constants.TURN_IDLEMODE[motorIndex]);
      sparkMotor.setSmartCurrentLimit(Constants.TURN_MAX_CURRENT_STALL, Constants.TURN_MAX_CURRENT_RUN);

      // create and initialize the PID for the heading
      anglePID = new PID(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);

      // get the initial error
      AngleProcessing();

      // set initial desired heading to the current actual heading.
      desiredAngle = currentAngle;

      // initially setup the PID parameters
      anglePID.setOutputLimits(Constants.OutputLowLimit, Constants.OutputHighLimit);
      anglePID.setMaxIOutput(Constants.MaxIOutput);
      anglePID.setOutputRampRate(Constants.OutputRampRate);
      anglePID.setOutputFilter(Constants.OutputFilter);
      anglePID.setSetpointRange(Constants.SetpointRange, Constants.SetpointRange);
      anglePID.setContinousInputRange(2 * Math.PI);  // sets circular continuous input range
      anglePID.setContinous(true);  // lets PID know we are working with a continuous range [0-360)

      // diagnostic print. comment out of production code
      //System.out.printf("desiredAngle: , currentAngle: %.4f , %.4f , %.4f  \n", desiredAngle, currentAngle, vTheta);

  }

  // process loop, called every execution cycle
  public void processTurn()
  {
    // get PID error signal to send to the motor
    AngleProcessing();

    // send to motor, signal -1 to 1
    sparkMotor.set(vTheta);

    // diagnostic print. comment out of production code
    System.out.printf("desiredAngle: , currentAngle: %.4f , %.4f , %.4f  \n", desiredAngle, currentAngle, vTheta);
  }

  // basic getter for angle.  Possible use for Dashboard
  public double getCurrentAngle()
  {
      return this.currentAngle;
  }


  // takes -PI to PI and processes the output to the motor controller.
  // This must be called repeatedly in the main robot loop.
  public void setDesiredAngle(double angle)
  {
    // convert to always positive angle between 0 and 2PI
    if (angle < 0)
    {
      angle = angle + (2 * Math.PI);
    }

    this.desiredAngle = angle;
  }


  // grab the current wheel angel and crunch out the value needed to correct to desired angle.
  // This method produces the angle input component to the motor from the PID that holds the
  // desired angle.  The error from the PID is sent to the motors in the vTheta variable.
  private void AngleProcessing()
  {
     // fetch the encoder ( +/- 1 = 1 rotation )
     // mod div to get number between (-2PI and 2PI)
      currentAngle = sparkEncoder.getPosition() % (2 * Math.PI);

      // always keep in terms of positive angle 0 to 2PI
      if (currentAngle < 0)
      {
          currentAngle = currentAngle + (2 * Math.PI);
      }

      vTheta = anglePID.getOutput(currentAngle, desiredAngle);
  }

  public void zeroEncoder()
  {
    sparkEncoder.setPosition(0.0);
  }

}
