package com.spartronics4915.lib.hardware.motors;

import com.spartronics4915.lib.util.Logger;

public interface SpartronicsEncoder {
    
    public final SpartronicsEncoder kDisconnectedEncoder = new SpartronicsEncoder() {

        @Override
        public void setPhase(boolean isReversed) {
            Logger.error("Couldn't set the phase of a disconnected encoder");
        }

        @Override
        public double getVelocity() {
            return 0;
        }

        @Override
        public double getPosition() {
            return 0;
        }
    };

    /**
     * @return Velocity in meters/second.
     */
    double getVelocity();

    /**
     * @return Position in meters.
     */
    double getPosition();

    /**
     * Sets the "direction" (phase) of this encoder.
     * 
     * @param isReversed If true, the sensor's output is reversed.
     */
    void setPhase(boolean isReversed);
}