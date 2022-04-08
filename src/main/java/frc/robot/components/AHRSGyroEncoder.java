package frc.robot.components;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.architecture.PositionEncoder;
import frc.robot.architecture.SpeedEncoder;

public class AHRSGyroEncoder implements PositionEncoder, SpeedEncoder {

    public final AHRS SENSOR;

    public AHRSGyroEncoder() {
        /* Communicate w/navX-MXP via the MXP SPI Bus. */
        /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
        /*
            * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.
            */
        
        SENSOR = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public double getCurrentPosition() {
        return fromNative(SENSOR.getAngle());
    }

    @Override
    public void setPosition(double newPosition) {
        SENSOR.setAngleAdjustment(toNative(newPosition) + SENSOR.getAngleAdjustment() - SENSOR.getAngle());
    }

    @Override
    public double getCurrentSpeed() {
        return fromNative(SENSOR.getRate());
    }

    private double toNative(double outputAngle) {
        return -Units.radiansToDegrees(outputAngle);
    }

    private double fromNative(double nativeAngle) {
        return -Units.degreesToRadians(nativeAngle);
    }
}
