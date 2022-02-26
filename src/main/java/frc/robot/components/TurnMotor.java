package frc.robot.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.CAN;
import frc.robot.Constants;
import frc.robot.architecture.AngleMotor;
import frc.robot.utilities.*;

public class TurnMotor implements AngleMotor {
  // heading about a unit circle in radians.
  private double currentAngle = 0.0; // rotates about the Z axis [0,2PI) rad.
  private double desiredAngle = 0.0; // rotates about the Z axis [0,2PI) rad.

  // private CANPIDController sparkPID;
  private CANSparkMax sparkMotor;
  private RelativeEncoder sparkEncoder;
  private DutyCycle throughboreDutyCycle;

  private PID anglePID = null;

  // error signal to motor range -1 to 1 based PID error with angle inputs ranging [0 to 2PI)
  private static double vTheta;

  public TurnMotor(int motorID, int motorIndex) {
    // sparkMotor = new CANSparkMax(motorID, MotorType.kBrushed); // Old cim brushed steer motor
    sparkMotor = new CANSparkMax(motorID, MotorType.kBrushless); // new Neo 550 brushless motor

    // sparkEncoder = sparkMotor.getEncoder(EncoderType.kQuadrature, 4096 * 6); // CTR encoder (one
    // with little magnet on the end of the shaft)
    // sparkEncoder = sparkMotor.getEncoder(EncoderType.kQuadrature, 8192 * 6); // New shaft encoder
    // Rev Through Bore Encoder version

    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the
    // setPositionConversionFactor
    sparkEncoder = sparkMotor.getEncoder(Type.kHallSensor, 42); // Spark Neo 550 motor built in
                                                                // encoder (need to do the external
                                                                // gear red)

    // sparkEncoder.setPositionConversionFactor(2 * Math.PI); // for the CRT and other shaft encoder

    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the
    // setPositionConversionFact
    // 6 to 1 for the steer gear and 10 to 1 for the gearbox on the little motor.
    sparkEncoder.setPositionConversionFactor(2 * Math.PI / 10); // encoder will return radians

    // zero the encoder on init to avoid haveing to power off the bot every time.
    throughboreDutyCycle = new DutyCycle(new DigitalInput(0));
    sparkEncoder.setPosition(throughboreDutyCycle.getOutput() * 2 * Math.PI);

    sparkMotor.setInverted(Constants.TURN_INVERT);
    sparkMotor.setIdleMode(Constants.TURN_IDLEMODE);
    sparkMotor.setSmartCurrentLimit(Constants.TURN_MAX_CURRENT_STALL,
        Constants.TURN_MAX_CURRENT_RUN);

    // create and initialize the PID for the heading
    anglePID = new PID(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D);

    // get the initial error
    AngleProcessing();

    // set initial desired heading to the current actual heading.
    desiredAngle = currentAngle;

    // initially setup the PID parameters
    anglePID.setOutputLimits(Constants.TURN_OUTPUT_LIMIT_LOW, Constants.TURN_OUTPUT_LIMIT_HIGH);
    anglePID.setMaxIOutput(Constants.TURN_MAX_I_OUT);
    anglePID.setOutputRampRate(Constants.TURN_OUTPUT_RAMPRATE);
    anglePID.setOutputFilter(Constants.TURN_OUTPUT_FILTER);
    anglePID.setSetpointRange(Constants.TURN_SETPOINT_RANGE);
    anglePID.setContinousInputRange(2 * Math.PI); // sets circular continuous input range
    anglePID.setContinous(true); // lets PID know we are working with a continuous range [0-360)

    // diagnostic print. comment out of production code
    // System.out.printf("desiredAngle: , currentAngle: %.4f , %.4f , %.4f \n", desiredAngle,
    // currentAngle, vTheta);

  }

  @Override
  public void init() {
    sparkEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    processTurn();
  }

  // process loop, called every execution cycle
  public void processTurn() {
    // get PID error signal to send to the motor
    AngleProcessing();

    // send to motor, signal -1 to 1
    sparkMotor.set(vTheta);

    // diagnostic print. comment out of production code
    // System.out.printf("desiredAngle: , currentAngle: %.4f , %.4f , %.4f \n", desiredAngle,
    // currentAngle, vTheta);
  }

  // basic getter for angle. Possible use for Dashboard
  public double getCurrentAngle() {
    return this.currentAngle;
  }


  // takes -PI to PI and processes the output to the motor controller.
  // This must be called repeatedly in the main robot loop.
  @Override
  public void setDesiredAngle(double angle) {
    // convert to always positive angle between 0 and 2PI
    if (angle < 0) {
      angle = angle + (2 * Math.PI);
    }

    this.desiredAngle = angle;
  }


  // grab the current wheel angel and crunch out the value needed to correct to desired angle.
  // This method produces the angle input component to the motor from the PID that holds the
  // desired angle. The error from the PID is sent to the motors in the vTheta variable.
  private void AngleProcessing() {
    // fetch the encoder ( +/- 1 = 1 rotation )
    // mod div to get number between (-2PI and 2PI)
    currentAngle = sparkEncoder.getPosition() % (2 * Math.PI);

    // always keep in terms of positive angle 0 to 2PI
    if (currentAngle < 0) {
      currentAngle = currentAngle + (2 * Math.PI);
    }

    vTheta = anglePID.getOutput(currentAngle, desiredAngle);
  }

  public void zeroEncoder() {
    sparkEncoder.setPosition(0.0);
  }

}
