package com.spartronics4915.lib.math.geometry;

import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.util.Util;

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

    protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

    public static final Twist2d identity()
    {
        return kIdentity;
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    public Twist2d()
    {
        this.dx = 0;
        this.dy = 0;
        this.dtheta = 0;
    }

    public Twist2d(double dx, double dy, double dtheta)
    {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public Twist2d(Twist2d src)
    {
        this.dx = src.dx;
        this.dy = src.dy;
        this.dtheta = src.dtheta;
    }

    public Twist2d scaled(double scale)
    {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
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
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }

    /**
     * for some applications interpolating a Twist2d may be fraught with peril. 
     * If you are trying to move from a Pose2d according to a Twist2d
     * consider using Pose2d.interpolate
     */
    @Override
    public Twist2d interpolate(final Twist2d other, double x)
    {
        if (x <= 0)
            return new Twist2d(this);
        else if (x >= 1)
            return new Twist2d(other);
        final Twist2d t = new Twist2d(
            this.dx + x*(other.dx - this.dx),
            this.dy + x*(other.dy - this.dy),
            this.dtheta + x*(other.dtheta - this.dtheta
            ));
        // should just return t, no need for scaled
        return t.scaled(x);
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }
}
