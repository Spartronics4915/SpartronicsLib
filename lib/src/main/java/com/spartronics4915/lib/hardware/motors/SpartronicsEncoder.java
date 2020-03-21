package com.spartronics4915.lib.hardware.motors;

public interface SpartronicsEncoder
{

    /**
     * @return Velocity in custom units/second.
     */
    double getVelocity();

    /**
     * @return Position in custom units.
     */
    double getPosition();

    /**
     * Sets the "direction" (phase) of this encoder.
     *
     * @param isReversed If true, the sensor's output is reversed.
     */
    void setPhase(boolean isReversed);

    /**
     * Sets the current position (The value stored, not PID target)
     *
     * @param TargetPosition position
     */
    boolean setPosition(double position);
}
