package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SparkMaxController implements Sendable {
    
    private CANSparkMax controller;
    private RelativeEncoder encoder;
    private boolean enabled;

    public SparkMaxController(int id) {
        controller = new CANSparkMax(id, MotorType.kBrushless);
        controller.set(0);
        encoder = controller.getEncoder();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Enabled", () -> enabled, (value) -> {
            enabled = value;
            controller.set(0.0);
        });
        builder.addBooleanProperty("Inverted", controller::getInverted, controller::setInverted);
        builder.addDoubleProperty("Power", controller::get, (value) -> {
            if (enabled)
                controller.set(value);
            else
                controller.set(0.0);
        });
        builder.addDoubleProperty("Position", encoder::getPosition, encoder::setPosition);
        builder.addDoubleProperty("Velocity", encoder::getVelocity, (value) -> {});
    }
}
