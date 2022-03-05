package frc.robot.architecture;

public interface SpeedEncoder {

    /**
     * Returns the current speed of the output shaft.
     * 
     * @return The speed the output shaft is running at in rad/s.
     */
    public double getCurrentSpeed();
    
}
