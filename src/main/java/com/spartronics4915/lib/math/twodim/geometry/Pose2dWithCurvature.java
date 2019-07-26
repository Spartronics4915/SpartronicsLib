package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.math.Util;

import java.text.DecimalFormat;

public class Pose2dWithCurvature implements Interpolable<Pose2dWithCurvature>
{

    private final Pose2d mPose;
    private final double mCurvature;
    private final double mDCurvatureDS;

    public Pose2dWithCurvature()
    {
        mPose = new Pose2d();
        mCurvature = 0.0;
        mDCurvatureDS = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature)
    {
        mPose = pose;
        mCurvature = curvature;
        mDCurvatureDS = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature, double dCurvatureDS)
    {
        mPose = pose;
        mCurvature = curvature;
        mDCurvatureDS = dCurvatureDS;
    }

    public Pose2dWithCurvature(final Translation2d translation, final Rotation2d rotation, double curvature)
    {
        mPose = new Pose2d(translation, rotation);
        mCurvature = curvature;
        mDCurvatureDS = 0.0;
    }

    public Pose2dWithCurvature(final Translation2d translation, final Rotation2d rotation, double curvature, double dCurvatureDS)
    {
        mPose = new Pose2d(translation, rotation);
        mCurvature = curvature;
        mDCurvatureDS = dCurvatureDS;
    }

    public final Pose2d getPose()
    {
        return mPose;
    }

    public Pose2dWithCurvature transformBy(Pose2d transform)
    {
        return new Pose2dWithCurvature(getPose().transformBy(transform), getCurvature(), getDCurvatureDs());
    }

    public Pose2dWithCurvature mirror()
    {
        return new Pose2dWithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
    }

    public double getCurvature()
    {
        return mCurvature;
    }

    public double getDCurvatureDs()
    {
        return mDCurvatureDS;
    }

    public final Translation2d getTranslation()
    {
        return getPose().getTranslation();
    }

    public final Rotation2d getRotation()
    {
        return getPose().getRotation();
    }

    @Override
    public Pose2dWithCurvature interpolate(final Pose2dWithCurvature other, double x)
    {
        return new Pose2dWithCurvature(getPose().interpolate(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    public double distance(final Pose2dWithCurvature other)
    {
        return getPose().distance(other.getPose());
    }

    public boolean equals(final Object other)
    {
        if (other == null || !(other instanceof Pose2dWithCurvature))
            return false;
        Pose2dWithCurvature p2dwc = (Pose2dWithCurvature) other;
        return getPose().equals(p2dwc.getPose()) && Util.epsilonEquals(getCurvature(), p2dwc.getCurvature())
                && Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    @Override
    public String toString()
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }
}
