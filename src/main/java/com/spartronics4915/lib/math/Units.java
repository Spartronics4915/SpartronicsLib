package com.spartronics4915.lib.math;

public class Units
{

    public static double rpm_to_rads_per_sec(double rpm)
    {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    public static double rads_per_sec_to_rpm(double rads_per_sec)
    {
        return rads_per_sec * 60.0 / (2.0 * Math.PI);
    }

    public static double inches_to_meters(double inches)
    {
        return inches * 0.0254;
    }

    public static double millimeters_to_inches(double millimeters)
    {
        return meters_to_inches(millimeters / 1000);
    }

    public static double inches_to_millimeters(double inches)
    {
        return inches_to_meters(inches) * 1000;
    }

    public static double meters_to_inches(double meters)
    {
        return meters / 0.0254;
    }

    public static double feet_to_meters(double feet)
    {
        return inches_to_meters(feet * 12.0);
    }

    public static double meters_to_feet(double meters)
    {
        return meters_to_inches(meters) / 12.0;
    }

    public static double degrees_to_radians(double degrees)
    {
        return Math.toRadians(degrees);
    }

    public static double radians_to_degrees(double radians)
    {
        return Math.toDegrees(radians);
    }
}
