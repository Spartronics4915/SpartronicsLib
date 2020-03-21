package com.spartronics4915.lib.hardware.motors;

import com.spartronics4915.lib.util.Logger;

public interface SpartronicsMotor
{

    /**
     * @return The encoder attached to the motor.
     */
    SpartronicsEncoder getEncoder();

    /**
     * @return A {@link SensorModel} that converts between native units and meters.
     */
    SensorModel getSensorModel();

    /**
     * This method is useful for detecting unplugged motor controllers.
     *
     * @return If true, the motor controller returned some type of error on startup.
     */
    boolean hadStartupError();

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
     * @param velocityCustomUnitsPerSecond The max velocity in custom units/s that the on board
     *                                motion profile generator will use.
     */
    void setMotionProfileCruiseVelocity(double velocityCustomUnitsPerSecond);

    /**
     * @return The max acceleration in custom units/s^2 that the on board motion profile
     *         generator will use.
     */
    double getMotionProfileMaxAcceleration();

    /**
     * @param accelerationCustomUnitsSecSq The max acceleration in custom units/s^2 that the on
     *                                board motion profile generator will use.
     */
    void setMotionProfileMaxAcceleration(double accelerationCustomUnitsSecSq);

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
    void setPercentOutput(double dutyCycle, double arbitraryFeedForwardVolts);

    /**
     * Sets the output as a percentage (like setOpenLoop).
     *
     * @param dutyCycle Output in percent.
     */
    void setPercentOutput(double dutyCycle);

    /**
     * Sets the target output velocity.
     *
     * @param velocityCustomUnitsPerSecond Velocity in custom units/s.
     */
    void setVelocity(double velocityCustomUnitsPerSecond);

    /**
     * Sets the target output velocity.
     *
     * @param velocityCustomUnitsPerSecond   Velocity in custom units/s.
     * @param arbitraryFeedForwardVolts Additional arbitrary feedforward in Volts.
     */
    void setVelocity(double velocityCustomUnitsPerSecond, double arbitraryFeedForwardVolts);

    /**
     * Sets the PD gains for the built in velocity PID controller.
     */
    void setVelocityGains(double kP, double kD);
    /**
     * Sets the PID gains for the built in velocity PID controller.
     */
    void setVelocityGains(double kP, double kI, double kD, double kF);

    /**
     * Sets the target position.
     *
     * @param positionCustomUnits Target position in custom units.
     */
    void setPosition(double positionCustomUnits);

    /**
     * Sets the PID gains for the built in position PID controller.
     */
    void setPositionGains(double kP, double kD);
    /**
     * Sets the PID gains for the built in position PID controller.
     */
    void setPositionGains(double kP, double kI, double kD, double kF);

    /**
     * Turns the motor off.
     */
    void setNeutral();

    /**
     * @return The output current of the motor in amps.
     */
    double getOutputCurrent();

    /**
     * @return The motor following this motor, or null.
     */
    SpartronicsMotor getFollower();

    /**
     * @return The device ID
     */
    int getDeviceNumber();

    /**
     * @param forwardLimitCustomUnits Forward soft limit position in custom units.
     * @param reverseLimitCustomUnits Reverse soft limit position in custom units.
     */
    void setSoftLimits(double forwardLimitCustomUnits, double reverseLimitCustomUnits);

    /**
     * @param limitAmps Max stator current in amps.
     */
    default void setStatorCurrentLimit(int limitAmps)
    {
        Logger.warning("Stator current limit not implemented for device number " + getDeviceNumber() + "!");
    }

    /**
     * @param limitAmps Max input current in amps.
     */
    default void setSupplyCurrentLimit(int limitAmps, double maxTimeAtLimit)
    {
        Logger.warning("Supply current limit not implemented for device number " + getDeviceNumber() + "!");
    }
}
