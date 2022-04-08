/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands
 * and command groups that allow control of the robot.
 */
public final class IO {

    private static final double AXIS_CLICK_THRESHOLD = 0.01;

    public enum Button {
        DriverA(DRIVER, XboxController.Button.kA.value),
        DriverB(DRIVER, XboxController.Button.kB.value),
        DriverX(DRIVER, XboxController.Button.kX.value),
        DriverY(DRIVER, XboxController.Button.kY.value),
        DriverLeftBumper(DRIVER, XboxController.Button.kLeftBumper.value),
        DriverRightBumper(DRIVER, XboxController.Button.kRightBumper.value),
        DriverLeftStick(DRIVER, XboxController.Button.kLeftStick.value),
        DriverRightStick(DRIVER, XboxController.Button.kRightStick.value),
        DriverStart(DRIVER, XboxController.Button.kStart.value),
        DriverBack(DRIVER, XboxController.Button.kBack.value),
        DriverLeftTrigger(IO.Axis.DriverLeftTrigger::get),
        DriverRightTrigger(IO.Axis.DriverRightTrigger::get),
        ManipulatorA(MANIPULATOR, XboxController.Button.kA.value),
        ManipulatorB(MANIPULATOR, XboxController.Button.kB.value),
        ManipulatorX(MANIPULATOR, XboxController.Button.kX.value),
        ManipulatorY(MANIPULATOR, XboxController.Button.kY.value),
        ManipulatorLeftBumper(MANIPULATOR, XboxController.Button.kLeftBumper.value),
        ManipulatorRightBumper(MANIPULATOR, XboxController.Button.kRightBumper.value),
        ManipulatorLeftStick(MANIPULATOR, XboxController.Button.kLeftStick.value),
        ManipulatorRightStick(MANIPULATOR, XboxController.Button.kRightStick.value),
        ManipulatorStart(MANIPULATOR, XboxController.Button.kStart.value),
        ManipulatorBack(MANIPULATOR, XboxController.Button.kBack.value),
        ManipulatorLeftTrigger(IO.Axis.ManipulatorLeftTrigger::get),
        ManipulatorRightTrigger(IO.Axis.ManipulatorRightTrigger::get);

        public final edu.wpi.first.wpilibj2.command.button.Button value;

        Button(GenericHID hid, int buttonId) {
            value = new JoystickButton(hid, buttonId);
        }

        Button(DoubleSupplier axis) {
            this(() -> axis.getAsDouble() > AXIS_CLICK_THRESHOLD);
        }

        Button(BooleanSupplier isPressed) {
            value = new edu.wpi.first.wpilibj2.command.button.Button(isPressed);
        }
    }

    public enum Axis {
        /** Increasing to the right */
        DriverLeftX(DRIVER::getLeftX),
        /** Increasing to the bottom */
        DriverLeftY(DRIVER::getLeftY),
        /** Increasing to the right */
        DriverRightX(DRIVER::getRightX),
        /** Increasing to the bottom */
        DriverRightY(DRIVER::getRightY),
        /** Positive only */
        DriverLeftTrigger(DRIVER::getLeftTriggerAxis),
        /** Positive only */
        DriverRightTrigger(DRIVER::getRightTriggerAxis),
        /** Increasing to the right */
        ManipulatorLeftX(MANIPULATOR::getLeftX),
        /** Increasing to the bottom */
        ManipulatorLeftY(MANIPULATOR::getLeftY),
        /** Increasing to the right */
        ManipulatorRightX(MANIPULATOR::getRightX),
        /** Increasing to the bottom */
        ManipulatorRightY(MANIPULATOR::getRightY),
        /** Positive only */
        ManipulatorLeftTrigger(MANIPULATOR::getLeftTriggerAxis),
        /** Positive only */
        ManipulatorRightTrigger(MANIPULATOR::getRightTriggerAxis);

        private DoubleSupplier valueSupplier;
        private DoubleSupplier outputSupplier;

        Axis(DoubleSupplier value) {
            this.valueSupplier = value;
            this.outputSupplier = valueSupplier;
        }

        private void addTransform(Function<Double, Double> transform) {
            this.outputSupplier = () -> transform.apply(this.valueSupplier.getAsDouble());
        }

        public double get() {
            return outputSupplier.getAsDouble();
        }
    }

    /** 
     * An enum holding references to joysticks which obey the robot coordinate system 
     * 
     * x is forward
     * y is leftward
     * angles are in radians increasing counter-clockwise
     */
    public enum Joystick {
        DriverLeft(() -> -IO.Axis.DriverLeftY.get(), () -> -IO.Axis.DriverLeftX.get(), true),
        DriverRight(() -> -IO.Axis.DriverRightY.get(), () -> -IO.Axis.DriverRightX.get(), true),
        DriverDPad(() -> -DRIVER.getPOV() != -1 ? Math.cos(Units.degreesToRadians(-DRIVER.getPOV())) : 0, () -> -DRIVER.getPOV() != -1 ? Math.sin(Units.degreesToRadians(-DRIVER.getPOV())) : 0, false),
        ManipulatorLeft(() -> -IO.Axis.ManipulatorLeftY.get(), () -> -IO.Axis.ManipulatorLeftX.get(), true),
        ManipulatorRight(() -> -IO.Axis.ManipulatorRightY.get(), () -> -IO.Axis.ManipulatorRightX.get(), true),
        ManipulatorDPad(() -> -MANIPULATOR.getPOV() != -1 ? Math.cos(Units.degreesToRadians(-MANIPULATOR.getPOV())) : 0, () -> -MANIPULATOR.getPOV() != -1 ? Math.sin(Units.degreesToRadians(-MANIPULATOR.getPOV())) : 0, false);

        private DoubleSupplier xSupplier, ySupplier;

        private double x, y;

        /**
         * Constructs a joystick with built-in normalization and a standard cooridnate system.
         * 
         * @param x forward [-1, 1]
         * @param y leftward [-1, 1]
         * @param remap whether to remap cooridates to a circle
         */
        Joystick(DoubleSupplier x, DoubleSupplier y, boolean remap) {
            if (remap) {
                xSupplier = () -> remap(x.getAsDouble(), y.getAsDouble());
                ySupplier = () -> remap(y.getAsDouble(), x.getAsDouble());
            } else {
                xSupplier = x;
                ySupplier = y;
            }
        }

        private void update() {
            x = xSupplier.getAsDouble();
            y = ySupplier.getAsDouble();
        }

        /**
         * @return the circular remapping of `a` with reference to `b`
         */
        private double remap(double a, double b) {
            return a * Math.sqrt(1 - b * b / 2);
        }

        /**
         * @return the displacement forward
         */
        public double getForward() {
            update();
            return x;
        }

        /**
         * @return the displacement leftward
         */
        public double getLeftward() {
            update();
            return y;
        }

        /**
         * @return the angle counter-clockwise from the x-axis [0, 2pi]
         */
        public double getAngle() {
            update();
            return Math.atan2(y, x) + y > 0 ? 0 : 2 * Math.PI;
        }

        public double getMagnitude() {
            update();
            return Math.sqrt(x*x + y*y);
        }
    }

    private static boolean initialized = false;

    public static final XboxController DRIVER = new XboxController(0);
    public static final XboxController MANIPULATOR = new XboxController(1);

    public IO() {
        if (!initialized) {
            Axis.DriverLeftX.addTransform(deadbandTransform(0.1).andThen(exponentialTransform(30)));
            Axis.DriverLeftY.addTransform(deadbandTransform(0.1).andThen(exponentialTransform(30)));
            Axis.DriverRightX.addTransform(deadbandTransform(0.1).andThen(exponentialTransform(30)));
            Axis.DriverRightY.addTransform(deadbandTransform(0.1).andThen(exponentialTransform(30)));
            Axis.ManipulatorLeftTrigger.addTransform(exponentialTransform(10));
            Axis.ManipulatorRightTrigger.addTransform(exponentialTransform(10));

            initialized = true;
        } else {
            System.err.println("IO has been instantiated more than once.");
        }
    }

    /** Create a transform which includes deadband.
     * 
     * f(x) = { x if |x| >= a else 0 }
     * 
     * @param deadband a
     */
    private Function<Double, Double> deadbandTransform(double deadband) {
        return (value) -> Math.abs(value) > deadband ? value : 0;
    }

    /** Create a transform which makes the magnitude exponential which retaining a range [-1, 1]
     * 
     * f(x) = x/|x| * (a^|x| - 1) / (a - 1)
     * 
     * @param exponent a
     */
    private Function<Double, Double> exponentialTransform(double exponent) {
        return (value) -> Math.signum(value) * ((Math.pow(exponent, Math.abs(value)) - 1) / (exponent - 1));
    }
}
