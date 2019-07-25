package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.util.Util;

/**
 * Represents a 2d pose (rigid transform) containing translational and
 * rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Pose2d implements IPose2d<Pose2d>
{

    protected static final Pose2d kIdentity = new Pose2d();

    public static final Pose2d identity()
    {
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final Translation2d mTranslation;
    protected final Rotation2d mRotation;

    public Pose2d()
    {
        mTranslation = new Translation2d();
        mRotation = new Rotation2d();
    }

    public Pose2d(double x, double y, final Rotation2d rotation)
    {
        mTranslation = new Translation2d(x, y);
        mRotation = rotation;
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation)
    {
        mTranslation = translation;
        mRotation = rotation;
    }

    public Pose2d(final Pose2d other)
    {
        mTranslation = new Translation2d(other.mTranslation);
        mRotation = new Rotation2d(other.mRotation);
    }

    public static Pose2d fromTranslation(final Translation2d translation)
    {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d rotation)
    {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     * 
     * See also ch 9 of:
     * http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
     */
    public static Pose2d exp(final Twist2d delta)
    {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps)
        {
            // small angle approximation
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        }
        else
        {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        Translation2d xlate = new Translation2d(delta.dx * s - delta.dy * c, 
                                                delta.dx * c + delta.dy * s);
        return new Pose2d(xlate,
                new Rotation2d(cos_theta, sin_theta, false));
    }

    /**
     * Logical inverse of the above: obtains a Twist2d from a delta-pose
     */
    public static Twist2d log(final Pose2d dPose)
    {
        final double dtheta = dPose.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = dPose.getRotation().cos() - 1.0;
        double halfCos; // halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps)
        {
            halfCos = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else
        {
            halfCos = -(half_dtheta * dPose.getRotation().sin()) / cos_minus_one;
        }
        final Translation2d transPart = dPose.getTranslation()
                .rotateBy(new Rotation2d(halfCos, -half_dtheta, false));
        return new Twist2d(transPart.x(), transPart.y(), dtheta);
    }

    @Override
    public Translation2d getTranslation()
    {
        return mTranslation;
    }

    @Override
    public Rotation2d getRotation()
    {
        return mRotation;
    }

    /**
     * Transforming this Pose2d means first translating by
     * other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public Pose2d transformBy(final Pose2d other)
    {
        return new Pose2d(mTranslation.translateBy(other.mTranslation.rotateBy(mRotation)),
                mRotation.rotateBy(other.mRotation));
    }

    /**
     * This transforms the pose directionally. E.g.:
     * if we're at 0, 0, 0 and we transform by 3,
     * we would be at 3, 0, 0.
     * 
     * @param scalar A scalar to transform this directionally by
     * @return This translated by the scalar in the direction of the rotation
     */
    public Pose2d transformBy(final double scalar)
    {
        return new Pose2d(
                this.getRotation().cos() * scalar + this.getTranslation().x(),
                this.getRotation().sin() * scalar + this.getTranslation().y(),
                this.getRotation());
    }

    /**
     * The inverse of this Pose2d "undoes" the effect of applying our transform.
     *
     * For p = new Pose2d(10, 10, -30deg)
     *     np = p.inverse()
     * 
     * @return The opposite of this transform.
     */
    public Pose2d inverse()
    {
        Rotation2d invRot = mRotation.inverse();
        return new Pose2d(mTranslation.inverse().rotateBy(invRot), invRot);
    }

    public Pose2d normal()
    {
        return new Pose2d(mTranslation, mRotation.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of
     * another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation2d intersection(final Pose2d other)
    {
        final Rotation2d other_rotation = other.getRotation();
        if (mRotation.isParallel(other_rotation))
        {
            // Lines are parallel.
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(mRotation.cos()) < Math.abs(other_rotation.cos()))
        {
            return intersectionInternal(this, other);
        }
        else
        {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2d other)
    {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2d twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon)
    {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    private static Translation2d intersectionInternal(final Pose2d a, final Pose2d b)
    {
        final Rotation2d a_r = a.getRotation();
        final Rotation2d b_r = b.getRotation();
        final Translation2d a_t = a.getTranslation();
        final Translation2d b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sin() - a_r.cos() * tan_b);
        if (Double.isNaN(t))
        {
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2d interpolate(final Pose2d other, double x)
    {
        if (x <= 0)
        {
            return new Pose2d(this);
        }
        else if (x >= 1)
        {
            return new Pose2d(other);
        }
        final Twist2d twist = Pose2d.log(inverse().transformBy(other));
        return transformBy(Pose2d.exp(twist.scaled(x)));
    }

    @Override
    public String toString()
    {
        return "T:" + mTranslation.toString() + ", R:" + mRotation.toString();
    }

    @Override
    public String toCSV()
    {
        return mTranslation.toCSV() + "," + mRotation.toCSV();
    }

    @Override
    public double distance(final Pose2d other)
    {
        return Pose2d.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Pose2d))
            return false;
        return epsilonEquals((Pose2d) other, Util.kEpsilon);
    }

    @Override
    public Pose2d getPose()
    {
        return this;
    }

    @Override
    public Pose2d mirror()
    {
        return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }
}
