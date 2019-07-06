package com.spartronics4915.lib.util;

public class Units
{

    public static double rpmToRadsPerSec(double rpm)
    {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    public static double radsPerSecToRpm(double rads_per_sec)
    {
        return rads_per_sec * 60.0 / (2.0 * Math.PI);
    }

    public static double inchesToMeters(double inches)
    {
        return inches * 0.0254;
    }

    public static double millimetersToInches(double millimeters)
    {
        return metersToInches(millimeters / 1000);
    }

    public static double inchesToMillimeters(double inches)
    {
        return inchesToMeters(inches) * 1000;
    }

    public static double metersToInches(double meters)
    {
        return meters / 0.0254;
    }

    public static double feetToMeters(double feet)
    {
        return inchesToMeters(feet * 12.0);
    }

    public static double metersToFeet(double meters)
    {
        return metersToInches(meters) / 12.0;
    }

    public static double degreesToRadians(double degrees)
    {
        return Math.toRadians(degrees);
    }

    public static double radiansToDegrees(double radians)
    {
        return Math.toDegrees(radians);
    }
}
