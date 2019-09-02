package com.spartronics4915.lib.hardware.motors;

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