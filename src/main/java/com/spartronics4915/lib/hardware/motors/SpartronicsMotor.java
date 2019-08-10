package com.spartronics4915.lib.hardware.motors;

public interface SpartronicsMotor {
    public class SensorModel {
        private final double mToMetersMultiplier;

        /**
         * This is a convenience constructor for sensors connected to a wheel. Use the
         * more direct constructor if your sensor is not connected to a wheel.
         * 
         * @param wheelDiameterMeters      The diameter of your wheel in meters.
         * @param nativeUnitsPerRevolution The number of meters per wheel revolution.
         */
        public SensorModel(double wheelDiameterMeters, double nativeUnitsPerRevolution) {
            mToMetersMultiplier = (1 / nativeUnitsPerRevolution) * (wheelDiameterMeters * Math.PI);
        }

        /**
         * @param nativeUnitsToMetersMultiplier A number that, when multiplied with some
         *                                      amount of meters, converts to meters.
         *                                      This factor will also be used to convert
         *                                      to related units, like meters/sec.
         */
        public SensorModel(double nativeUnitsToMetersMultiplier) {
            mToMetersMultiplier = nativeUnitsToMetersMultiplier;
        }

        public double toMeters(double nativeUnits) {
            return nativeUnits * mToMetersMultiplier;
        }

        public double toNativeUnits(double meters) {
            return meters / mToMetersMultiplier;
        }
    }

    /**
     * @return The encoder attached to the motor. Null if none is attached.
     */
    SpartronicsEncoder getEncoder();

    /**
     * @return A {@link SensorModel} that converts between meters and meters.
     */
    SensorModel getSensorModel();

    /**
     * @return Current output of the motor controller in Volts.
     */
    double getVoltageOutput();

    /**
     * @return Is the motor output inverted.
     */
    boolean getOutputInverted();

    /**
     * @param inverted If true, the motor output is inverted.
     */
    void setOutputInverted(boolean inverted);

    /**
     * @return Are the motor leads electrically commonized to reduce movement.
     */
    boolean getBrakeMode();

    /**
     * @param mode If true, commonize the motor leads to reduce movement.
     */
    void setBrakeMode(boolean mode);

    /**
     * @return The max voltage output given to the motor.
     */
    double getVoltageCompSaturation();

    /**
     * @param voltage Sets the max voltage output given to the motor, in Volts.
     */
    void setVoltageCompSaturation(double voltage);

    /**
     * @return The max velocity in meters/s that the on board motion profile
     *         generator will use.
     */
    double getMotionProfileCruiseVelocity();

    /**
     * @param velocityMetersPerSecond The max velocity in meters/s that the on board
     *                          motion profile generator will use.
     */
    void setMotionProfileCruiseVelocity(double velocityMetersPerSecond);

    /**
     * @return The max acceleration in meters/s^2 that the on board motion profile
     *         generator will use.
     */
    double getMotionProfileMaxAcceleration();

    /**
     * @param accelerationMetersSecSq The max acceleration in meters/s^2 that the on
     *                                board motion profile generator will use.
     */
    void setMotionProfileMaxAcceleration(double accelerationMetersSecSq);

    /**
     * @param useMotionProfile If true, we will use the motion profile to get to
     *                         positions passed to
     *                         {@link SpartronicsMotor#setPosition(double)
     *                         setPosition}.
     */
    void setUseMotionProfileForPosition(boolean useMotionProfile);

    /**
     * Sets the output as a percentage (like setOpenLoop).
     * 
     * @param dutyCycle            Output in perecnt.
     * @param arbitraryFeedforward Additional arbitrary feedforward in Volts.
     */
    void setDutyCycle(double dutyCycle, double arbitraryFeedForwardVolts);

    /**
     * Sets the output as a percentage (like setOpenLoop).
     * 
     * @param dutyCycle Output in percent.
     */
    void setDutyCycle(double dutyCycle);

    /**
     * Sets the target output velocity.
     * 
     * @param velocityMetersPerSecond Velocity in meters/s.
     */
    void setVelocity(double velocityMetersPerSecond);

    /**
     * Sets the target output velocity.
     * 
     * @param velocityMetersPerSecond    Velocity in meters/s.
     * @param arbitraryFeedForwardVolts Additional arbitrary feedforward in Volts.
     */
    void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts);

    /**
     * Sets the PID gains for the built in velocity PID controller.
     */
    void setVelocityGains(double kP, double kI, double kD, double kF);

    /**
     * Sets the target position.
     * 
     * @param positionMeters Target position in meters.
     */
    void setPosition(double positionMeters);

    /**
     * Sets the PID gains for the built in position PID controller.
     */
    void setPositionGains(double kP, double kI, double kD, double kF);

    /**
     * Turns the motor off.
     */
    void setNeutral();
}