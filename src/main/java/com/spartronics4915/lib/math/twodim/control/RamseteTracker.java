package com.spartronics4915.lib.math.twodim.control;

import com.spartronics4915.lib.math.Util;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedIterator;

/**
 * Uses a time-varying non linear reference controller to steer the robot back
 * onto the trajectory.
 * 
 * From https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf eq 5.12
 * 
 * https://file.tavsys.net/control/state-space-guide.pdf section 8.5 is also
 * helpful and a little more accessible
 */
public class RamseteTracker extends TrajectoryTracker {
    // These terms don't work out dimensionally
    // But whatever, it works
    private final double kBeta, kZeta;

    /**
     * @param beta Constant for correction. Increase for more aggresive convergence.
     *             beta > 0
     * @param zeta Dampening constant. Increase for more dampening. zeta ∈ (0,1)
     */
    public RamseteTracker(double beta, double zeta) {
        kBeta = beta;
        kZeta = zeta;
    }

	@Override
	protected TrajectoryTrackerVelocityOutput calculateState(
        TimedIterator<Pose2dWithCurvature> iterator,
        Pose2d currentRobotPoseMeters
    ) {
        var referenceState = iterator.getCurrentSample().state;

        // This can be thought of as the goal pose in robot coordinates
        Pose2d error = referenceState.state.getPose().inFrameReferenceOf(currentRobotPoseMeters);

        // Get reference linear and angular velocities (meters/sec, rads/sec)
        double vd = referenceState.velocity;
        double wd = vd * referenceState.state.getCurvature();

        // Compute the first time-varying gain
        double k1 = 2 * kZeta * Math.sqrt(wd * wd + kBeta * vd * vd);

        // Get bounded angular error (radians)
        double angleError = error.getRotation().getRadians();

        return new TrajectoryTrackerVelocityOutput(
            vd * error.getRotation().cos() + k1 * error.getTranslation().x(),
            wd + kBeta * vd * sinc(angleError) * error.getTranslation().y() + k1 * angleError
        );
    }
    
    private double sinc(double theta) {
        if (Util.epsilonEquals(theta, 0)) {
            return 1 - 1 / 6 * theta * theta;
        } else {
            return Math.sin(theta) / theta;
        }
    }

}