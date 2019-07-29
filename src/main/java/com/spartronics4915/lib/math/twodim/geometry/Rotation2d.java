package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.math.Util;

import java.text.DecimalFormat;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 */
public class Rotation2d implements Interpolable<Rotation2d>
{

    private final double mCosAngle;
    private final double mSinAngle;

    public Rotation2d()
    {
        this(1, 0, false);
    }

    public Rotation2d(double x, double y, boolean normalize)
    {
        if (normalize)
        {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = Math.hypot(x, y);
            if (magnitude > Util.kEpsilon)
            {
                mSinAngle = y / magnitude;
                mCosAngle = x / magnitude;
            }
            else
            {
                mSinAngle = 0;
                mCosAngle = 1;
            }
        }
        else
        {
            mCosAngle = x;
            mSinAngle = y;
        }
    }

    public Rotation2d(final Rotation2d other)
    {
        mCosAngle = other.mCosAngle;
        mSinAngle = other.mSinAngle;
    }

    public Rotation2d(final Translation2d direction, boolean normalize)
    {
        this(direction.x(), direction.y(), normalize);
    }

    public static Rotation2d fromRadians(double angleRadians)
    {
        return new Rotation2d(Math.cos(angleRadians), Math.sin(angleRadians), false);
    }

    public static Rotation2d fromDegrees(double angleDegrees)
    {
        return fromRadians(Math.toRadians(angleDegrees));
    }

    public double cos()
    {
        return mCosAngle;
    }

    public double sin()
    {
        return mSinAngle;
    }

    public double tan()
    {
        if (Math.abs(mCosAngle) < Util.kEpsilon)
        {
            if (mSinAngle >= 0.0)
            {
                return Double.POSITIVE_INFINITY;
            }
            else
            {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return mSinAngle / mCosAngle;
    }

    public double getRadians()
    {
        return Math.atan2(mSinAngle, mCosAngle);
    }

    public double getDegrees()
    {
        return Math.toDegrees(getRadians());
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and
     * another rotation.
     *
     * @param other The other rotation. See:
     *              https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation2d rotateBy(final Rotation2d other)
    {
        return new Rotation2d(mCosAngle * other.mCosAngle - mSinAngle * other.mSinAngle,
                mCosAngle * other.mSinAngle + mSinAngle * other.mCosAngle, true);
    }

    public Rotation2d normal()
    {
        return new Rotation2d(-mSinAngle, mCosAngle, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public Rotation2d inverse()
    {
        return new Rotation2d(mCosAngle, -mSinAngle, false);
    }

    public boolean isParallel(final Rotation2d other)
    {
        return Util.epsilonEquals(Translation2d.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation2d toTranslation()
    {
        return new Translation2d(mCosAngle, mSinAngle);
    }

    @Override
    public Rotation2d interpolate(final Rotation2d endValue, double t)
    {
        if (t <= 0)
        {
            return new Rotation2d(this);
        }
        else if (t >= 1)
        {
            return new Rotation2d(endValue);
        }
        double angleDiff = inverse().rotateBy(endValue).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angleDiff * t));
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(getDegrees()) + " deg)";
    }

    public double distance(final Rotation2d other)
    {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Rotation2d))
            return false;
        return distance((Rotation2d) other) < Util.kEpsilon;
    }

    public Rotation2d getRotation()
    {
        return this;
    }
}
