package com.spartronics4915.lib.hardware.motors;

public class SensorModel {
    private final double mToCustomUnitsMultiplier;

    /**
     * This is a convenience constructor for sensors connected to a wheel. Use the
     * more direct constructor if your sensor is not connected to a wheel.
     * 
     * @param wheelDiameterMeters      The diameter of your wheel in custom units.
     * @param nativeUnitsPerRevolution The number of custom units per wheel revolution.
     */
    public static SensorModel fromWheelDiameter(double wheelDiameterMeters, double nativeUnitsPerRevolution) {
        return new SensorModel((1 / nativeUnitsPerRevolution) * (wheelDiameterMeters * Math.PI));
    }

    /**
     * @param nativeUnitsToCustomUnitsMultiplier A number that, when multiplied with some
     *                                           amount of custom units, converts to meters.
     *                                           This factor will also be used to convert
     *                                           to related units, like custom units/sec.
     */
    public static SensorModel fromMultiplier(double nativeUnitsToCustomUnitsMultiplier) {
        return new SensorModel(nativeUnitsToCustomUnitsMultiplier);
    }

    private SensorModel(double nativeUnitsToMetersMultiplier) {
        mToCustomUnitsMultiplier = nativeUnitsToMetersMultiplier;
    }

    public double toCustomUnits(double nativeUnits) {
        return nativeUnits * mToCustomUnitsMultiplier;
    }

    public double toNativeUnits(double meters) {
        return meters / mToCustomUnitsMultiplier;
    }
}