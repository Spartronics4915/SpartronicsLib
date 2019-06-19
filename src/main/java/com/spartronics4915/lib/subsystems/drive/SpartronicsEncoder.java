package com.spartronics4915.lib.subsystems.drive;

public interface SpartronicsEncoder
{
    /**
     * @return Velocity in native units/100ms.
     */
    double getVelocity();

    /**
     * @return Position in native units.
     */
    double getPosition();

    /**
     * Sets the "direction" (phase) of this encoder.
     * 
     * @param isReversed If true, the sensor's output is reversed.
     */
    void setPhase(boolean isReversed);
}