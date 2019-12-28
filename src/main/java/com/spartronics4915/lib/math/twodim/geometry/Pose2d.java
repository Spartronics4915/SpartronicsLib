package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.math.Util;
import com.spartronics4915.lib.math.twodim.trajectory.types.State;

/**
 * Represents a 2d pose (rigid transform) containing translational and
 * rotational elements.
 */
public class Pose2d implements State<Pose2d>
{

    private final Translation2d mTranslation;
    private final Rotation2d mRotation;

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
     * Logical inverse of the above: obtains a Twist2d from a delta-pose
     */
    public Twist2d log()
    {
        final double dtheta = getRotation().getRadians();
        final double halfDtheta = 0.5 * dtheta;
        final double cosMinusOne = getRotation().getCos() - 1.0;
        double halfCos; // halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cosMinusOne) < Util.kEpsilon)
        {
            halfCos = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else
        {
            halfCos = -(halfDtheta * getRotation().getSin()) / cosMinusOne;
        }
        final Translation2d transPart = getTranslation()
                .rotateBy(new Rotation2d(halfCos, -halfDtheta, false));
        return new Twist2d(transPart.getX(), transPart.getY(), Rotation2d.fromRadians(dtheta));
    }

    public Translation2d getTranslation()
    {
        return mTranslation;
    }

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
                this.getRotation().getCos() * scalar + this.getTranslation().getX(),
                this.getRotation().getSin() * scalar + this.getTranslation().getY(),
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

    public Pose2d inFrameReferenceOf(Pose2d fieldRelativeOrigin) {
        return fieldRelativeOrigin.inverse().transformBy(this);
    }

    public Pose2d normal()
    {
        return new Pose2d(mTranslation, mRotation.normal());
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2d other)
    {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2d twist = inverse().transformBy(other).log();
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta.getRadians(), 0.0));
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon)
    {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2d interpolate(final Pose2d endValue, double t)
    {
        if (t <= 0)
        {
            return new Pose2d(this);
        }
        else if (t >= 1)
        {
            return new Pose2d(endValue);
        }
        final Twist2d twist = inverse().transformBy(endValue).log();
        return transformBy(twist.scaled(t).exp());
    }

    @Override
    public String toString()
    {
        return "T:" + mTranslation.toString() + ", R:" + mRotation.toString();
    }

    @Override
    public double distance(final Pose2d other)
    {
        return inverse().transformBy(other).log().norm();
    }

    @Override
    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Pose2d))
            return false;
        return epsilonEquals((Pose2d) other, Util.kEpsilon);
    }

    public Pose2d getPose()
    {
        return this;
    }

    public Pose2d mirror()
    {
        return new Pose2d(new Translation2d(getTranslation().getX(), -getTranslation().getY()), getRotation().inverse());
    }
}
