package com.spartronics4915.lib.math.twodim.spline;

import com.spartronics4915.lib.math.twodim.geometry.*;
import com.spartronics4915.lib.util.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class SplineGenerator
{
    private static final int kMinSampleSize = 1;
    private static final int kRecursionDepthLimit = 500;
    
    /**
     * Converts a spline into a list of Pose2dWithCurvatures
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    public static List<Pose2dWithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, Rotation2d maxDTheta, double t0, double t1)
    {
        List<Pose2dWithCurvature> rv = new ArrayList<>();
        rv.add(s.getPose2dWithCurvature(0.0));
        double dt = (t1 - t0) / kMinSampleSize;
        for (double t = 0; t < t1; t += dt)
        {
            getSegmentArc(s, new AtomicInteger(0), rv, t, t + dt, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    public static List<Pose2dWithCurvature> parameterizeSplines(List<? extends Spline> splines, double maxDx, double maxDy,
            Rotation2d maxDTheta)
    {
        List<Pose2dWithCurvature> rv = new ArrayList<>();
        if (splines.isEmpty())
            return rv;
        rv.add(splines.get(0).getPose2dWithCurvature(0.0));
        for (final Spline s : splines)
        {
            List<Pose2dWithCurvature> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0, 1);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    // We have to use AtomicInteger because int and Integer are pass-by-value
    // We do a depth count because this can crash your robot with a stack overflow if you attempt to parameterize an invalid spline (Java doesn't do tail call optimization)
    private static void getSegmentArc(Spline s, AtomicInteger depthCount, List<Pose2dWithCurvature> rv, double t0, double t1, double maxDx,
            double maxDy,
            Rotation2d maxDTheta)
    {
        Translation2d p0 = s.getPoint(t0);
        Translation2d p1 = s.getPoint(t1);
        Rotation2d r0 = s.getHeading(t0);
        Rotation2d r1 = s.getHeading(t1);
        Pose2d transformation = new Pose2d(new Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        Twist2d twist = transformation.log();

        if (Math.abs(twist.dy) > maxDy || Math.abs(twist.dx) > maxDx || Math.abs(twist.dtheta.getRadians()) > maxDTheta.getRadians())
        {
            if (depthCount.incrementAndGet() > kRecursionDepthLimit)
            {
                throw new RuntimeException("Hit recursion depth limit!\ntwist: " + twist.toString() + ", p0: " + p0 + ", p1: " + p1 + ", r0: " + r0 + ", r1: " + r1 + ", transformation: " + transformation);
            }

            // subdivide
            getSegmentArc(s, depthCount, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, depthCount, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        }
        else
        {
            rv.add(s.getPose2dWithCurvature(t1));
        }
    }

}
