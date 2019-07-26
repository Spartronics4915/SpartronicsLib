package com.spartronics4915.lib.math;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util
{

    public static final double kEpsilon = 1e-9;

    private Util()
    {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude)
    {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max)
    {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x)
    {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b)
    {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon)
    {
        boolean result = true;
        for (Double value_in : list)
        {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }
}
