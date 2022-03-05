package frc.robot.architecture;

public interface PositionEncoder {



    /**
     * Returns the current position of the output shaft.
     * 
     * @return The current position of the output shaft in radians.
     */
    public double getCurrentPosition();

    /**
     * Returns the current heading of the output shaft.
     * 
     * @return The current angle the output shaft is facing in radians [0, 2PI).
     */
    public default double getCurrentAngle() {
        return getCurrentPosition() % (2* Math.PI);
    };
    
    /**
     * Sets the current position of the output shaft to be 0.
     */
    public void setPosition(double newPosition);
}
