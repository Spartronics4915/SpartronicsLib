package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.math.Util;

import java.text.DecimalFormat;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas
 * from "differential calculus" to create
 * new Pose2ds from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity,
 * an acceleration, etc.
 */
public class Twist2d implements Interpolable<Twist2d>
{

    public final double dx;
    public final double dy;
    public final Rotation2d dtheta;

    private final double mDThetaRads;

    public Twist2d()
    {
        this.dx = 0;
        this.dy = 0;
        this.dtheta = new Rotation2d();
        mDThetaRads = 0;
    }

    public Twist2d(double dx, double dy, Rotation2d dtheta)
    {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
        mDThetaRads = dtheta.getRadians();
    }

    public Twist2d(Twist2d src)
    {
        this.dx = src.dx;
        this.dy = src.dy;
        this.dtheta = src.dtheta;
        mDThetaRads = src.mDThetaRads;
    }

    public Twist2d scaled(double scale)
    {
        return new Twist2d(dx * scale, dy * scale, Rotation2d.fromRadians(mDThetaRads * scale));
    }

    public double norm()
    {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double curvature()
    {
        if (Math.abs(mDThetaRads) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return mDThetaRads / norm();
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     * 
     * See also ch 9 of:
     * http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
     */
    public Pose2d exp() {
        double sin_theta = this.dtheta.getSin();
        double cos_theta = this.dtheta.getCos();
        double s, c;
        if (Math.abs(mDThetaRads) < Util.kEpsilon) {
            // small angle approximation
            s = 1.0 - 1.0 / 6.0 * mDThetaRads * mDThetaRads;
            c = .5 * mDThetaRads;
        } else {
            s = sin_theta / mDThetaRads;
            c = (1.0 - cos_theta) / mDThetaRads;
        }
        Translation2d xlate = new Translation2d(this.dx * s - this.dy * c, this.dx * c + this.dy * s);
        return new Pose2d(xlate, new Rotation2d(cos_theta, sin_theta, false));
    }

    /**
     * For some applications interpolating a Twist2d may be fraught with peril. 
     * If you are trying to move from a Pose2d according to a Twist2d
     * consider using Pose2d.interpolate
     */
    @Override
    public Twist2d interpolate(final Twist2d endValue, double t)
    {
        if (t <= 0)
            return this;
        else if (t >= 1)
            return new Twist2d(endValue);
        final Twist2d newTwist = new Twist2d(
            this.dx + t*(endValue.dx - this.dx),
            this.dy + t*(endValue.dy - this.dy),
            Rotation2d.fromRadians(mDThetaRads + t*(endValue.mDThetaRads - this.mDThetaRads)));
        // should just return t, no need for scaled
        return newTwist.scaled(t);
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(dtheta.getDegrees()) + " deg)";
    }
}
