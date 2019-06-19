package com.spartronics4915.lib.subsystems.drive;

public interface SpartronicsMotor
{
    /**
     * @return The encoder attached to the motor. Null if none is attached.
     */
    SpartronicsEncoder getEncoder();

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
     * @return The max velocity in native units/s that the on board motion profile generator will use.
     */
    double getMotionProfileCruiseVelocity();

    /**
     * @param velocity The max velocity in native units/s that the on board motion profile generator will use.
     */
    void setMotionProfileCruiseVelocity(double velocity);

    /**
     * @return The max acceleration in native units/s^2 that the on board motion profile generator will use.
     */
    double getMotionProfileMaxAcceleration();

    /**
     * @param acceleration The max acceleration in native units/s^2 that the on board motion profile generator will use.
     */
    void setMotionProfileMaxAcceleration(double acceleration);

    /**
     * @param useMotionProfile If true, we will use the motion profile to get to positions passed to {@link SpartronicsMotor#setPosition(double) setPosition}.
     */
    void setUseMotionProfileForPosition(boolean useMotionProfile);

    /**
     * Sets the output as a percentage (like setOpenLoop).
     * 
     * @param dutyCycle Output in perecnt.
     * @param arbitraryFeedforward Additional arbitrary feedforward in Volts.
     */
    void setDutyCycle(double dutyCycle, double arbitraryFeedForward);

    /**
     * Sets the output as a percentage (like setOpenLoop).
     * 
     * @param dutyCycle Output in percent.
     */
    void setDutyCycle(double dutyCycle);

    /**
     * Sets the target output velocity.
     * 
     * @param velocity Velocity in native units/s.
     * @param arbitraryFeedForward Additional arbitrary feedforward in Volts.
     */
    void setVelocity(double velocity, double arbitraryFeedForward);

    /**
     * Sets the PID gains for the built in velocity PID controller.
     */
    void setVelocityGains(double kP, double kI, double kD, double kF);

    /**
     * Sets the target position.
     * 
     * @param position Target position in native units.
     */
    void setPosition(double position);

    /**
     * Sets the PID gains for the built in position PID controller.
     */
    void setPositionGains(double kP, double kI, double kD, double kF);
}