package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.math.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an
 * (x, y) plane.
 */
public class Translation2d implements Interpolable<Translation2d>
{

    private final double mX;
    private final double mY;

    public Translation2d()
    {
        mX = 0;
        mY = 0;
    }

    public Translation2d(double x, double y)
    {
        mX = x;
        mY = y;
    }

    public Translation2d(final Translation2d other)
    {
        mX = other.mX;
        mY = other.mY;
    }

    public Translation2d(final Translation2d start, final Translation2d end)
    {
        mX = end.mX - start.mX;
        mY = end.mY - start.mY;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm()
    {
        return Math.hypot(mX, mY);
    }

    public double norm2()
    {
        return mX * mX + mY * mY;
    }

    public double x()
    {
        return mX;
    }

    public double y()
    {
        return mY;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation2d translateBy(final Translation2d other)
    {
        return new Translation2d(mX + other.mX, mY + other.mY);
    }

    /**
     * We can also rotate Translation2d's. See:
     * https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2d rotateBy(final Rotation2d rotation)
    {
        return new Translation2d(mX * rotation.cos() - mY * rotation.sin(), mX * rotation.sin() + mY * rotation.cos());
    }

    public Rotation2d direction()
    {
        return new Rotation2d(mX, mY, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation2d inverse()
    {
        return new Translation2d(-mX, -mY);
    }

    @Override
    public Translation2d interpolate(final Translation2d other, double x)
    {
        if (x <= 0)
        {
            return new Translation2d(this);
        }
        else if (x >= 1)
        {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    public Translation2d extrapolate(final Translation2d other, double x)
    {
        return new Translation2d(x * (other.mX - mX) + mX, x * (other.mY - mY) + mY);
    }

    public Translation2d scale(double s)
    {
        return new Translation2d(mX * s, mY * s);
    }

    public boolean epsilonEquals(final Translation2d other, double epsilon)
    {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(mX) + "," + fmt.format(mY) + ")";
    }

    public static double dot(final Translation2d a, final Translation2d b)
    {
        return a.mX * b.mX + a.mY * b.mY;
    }

    public static Rotation2d getAngle(final Translation2d a, final Translation2d b)
    {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle))
        {
            return new Rotation2d();
        }
        return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    public static double cross(final Translation2d a, final Translation2d b)
    {
        return a.mX * b.mY - a.mY * b.mX;
    }

    public double distance(final Translation2d other)
    {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Translation2d))
            return false;
        return distance((Translation2d) other) < Util.kEpsilon;
    }

    public Translation2d getTranslation()
    {
        return this;
    }
}
