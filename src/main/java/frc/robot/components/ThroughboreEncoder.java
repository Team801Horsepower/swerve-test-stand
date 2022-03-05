package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.architecture.PositionEncoder;
import frc.robot.utilities.Utils;

public class ThroughboreEncoder implements PositionEncoder {

    public static final String PART_NAME = "REV-11-1271";

    public final int DIO_PIN;

    private final DutyCycle DUTY_CYCLE;

    private double offset;

    public ThroughboreEncoder(int dioPin) {
        DIO_PIN = dioPin;
        DUTY_CYCLE = new DutyCycle(new DigitalInput(dioPin));

        offset = Preferences.getDouble(PART_NAME + "[" + DIO_PIN + "]/offset", 0.0);
    }

    @Override
    public double getCurrentPosition() {
        return Utils.normalizeAngle(DUTY_CYCLE.getOutput() * 2 * Math.PI - offset);
    }

    @Override
    public void setPosition(double newPosition) {
        offset = newPosition;
        Preferences.setDouble(PART_NAME + "[" + DIO_PIN + "]/offset", DUTY_CYCLE.getOutput() * 2 * Math.PI - newPosition);
    }
}
