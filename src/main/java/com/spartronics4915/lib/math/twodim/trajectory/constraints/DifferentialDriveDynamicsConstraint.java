package com.spartronics4915.lib.math.twodim.trajectory.constraints;

import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;

public class DifferentialDriveDynamicsConstraint implements TimingConstraint<Pose2dWithCurvature> {

    private final DifferentialDrive mDifferentialDrive;
    /** Non-normalized volts (i.e. this is in the domain of [0, 12]) */
    private final double mMaxVoltage;

    public DifferentialDriveDynamicsConstraint(DifferentialDrive drive, double maxVoltage) {
        mDifferentialDrive = drive;
        mMaxVoltage = maxVoltage;
    }

    @Override
    public double getMaxVelocity(Pose2dWithCurvature state) {
        return mDifferentialDrive.getMaxAbsVelocity(state.getCurvature(), mMaxVoltage);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithCurvature state, double velocity) {
        DifferentialDrive.MinMax minMax = mDifferentialDrive.getMinMaxAcceleration(
            new DifferentialDrive.ChassisState(velocity, velocity * state.getCurvature()),
            state.getCurvature(),
            mMaxVoltage
        );

        return new MinMaxAcceleration(minMax.min, minMax.max);
    }

}