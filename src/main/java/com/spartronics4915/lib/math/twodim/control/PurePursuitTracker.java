package com.spartronics4915.lib.math.twodim.control;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedIterator;

/**
 * Uses an adaptive pure pursuit controller to steer the robot back onto the
 * desired trajectory. From
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 */
public class PurePursuitTracker extends TrajectoryTracker {

    private final double kLat;
    /** Seconds */
    private final double kLookaheadTime;
    /** Meters */
    private final double kMinLookaheadDistance;

    /**
     * @param lat                        Latitudinal error gain. This is effectively a
     *                                   "P" term in a PID controller. You tune this
     *                                   about the same way you would tune P
     *                                   normally.
     * @param lookaheadTimeSeconds       Lookahead time in seconds. Larger values
     *                                   mean slower but more stable convergence.
     * @param minLookaheadDistanceMeters Minimum lookahead distance in meters. Helps
     *                                   with stability, mostly when you're near the
     *                                   end of the path.
     */
    public PurePursuitTracker(double lat, double lookaheadTimeSeconds, double minLookaheadDistanceMeters) {
        kLat = lat;
        kLookaheadTime = lookaheadTimeSeconds;
        kMinLookaheadDistance = minLookaheadDistanceMeters;
    }

    @Override
    protected TrajectoryTrackerVelocityOutput calculateState(TimedIterator<Pose2dWithCurvature> iterator, Pose2d currentRobotPoseMeters) {
        // Target point on the path for the current time
        var referencePoint = iterator.getCurrentSample();

        // Field-relative pose of the lookahead point
        Pose2d lookaheadState = calculateLookaheadPose(iterator, currentRobotPoseMeters);

        // Robot-relative transform needed to get to the lookaheda
        Pose2d lookaheadTransform = lookaheadState.inFrameReferenceOf(currentRobotPoseMeters);

        // Calculate latitudinal error
        double xError = (referencePoint.state.state.getPose().inFrameReferenceOf(currentRobotPoseMeters).getTranslation().getX());

        // Calculate the velocity at the reference point (meters/sec)
        double vd = referencePoint.state.velocity;

        // Distance from the robot to the lookahead (meters)
        double l = lookaheadTransform.getTranslation().norm();

        // Calculate the curvature of the arc that connects the robot and the lookahead point
        double curvature = 2 * lookaheadTransform.getTranslation().getY() / Math.pow(l, 2);

        // Adjust the linear velocity proportional to the product of a gain and the error, and the rotational error
        double adjustedLinearVelocity = vd * lookaheadTransform.getRotation().getCos() + kLat * xError;

        return new TrajectoryTrackerVelocityOutput(
            adjustedLinearVelocity,
            adjustedLinearVelocity * curvature // velocity * curvature = angular velocity
        );
    }

    private Pose2d calculateLookaheadPose(TimedIterator<Pose2dWithCurvature> iterator, Pose2d robotPose) {
        Pose2d lookaheadPoseByTime = iterator.preview(kLookaheadTime).state.state.getPose();

        // If the lookahead point is farther from the robot than the min lookahead
        // distance we just use the lookaheadPoseByTime.
        if (lookaheadPoseByTime.inFrameReferenceOf(robotPose).getTranslation().norm() >= kMinLookaheadDistance) {
            return lookaheadPoseByTime;
        }

        Pose2d lookaheadPoseByDistance = iterator.getCurrentSample().state.state.getPose();
        double previewedTime = kLookaheadTime; // Seconds

        // Preview forwards in the trajectory until the distance from the robot to the
        // previewed point is greater than the minimum lookahead distance. If we reach
        // the end of the trajectry then we just extend the trajectory in the direction
        // of the final pose.
        while (iterator.getRemainingProgress() > previewedTime) {
            previewedTime += 0.02;

            // Remember that TrajectoryIterator::preview previews in an additive manner
            // E.g. in this situation iterator.preview(3) would preview at iterator.getProgress() + 3
            lookaheadPoseByDistance = iterator.preview(previewedTime).state.state.getPose();
            double lookaheadDistance = lookaheadPoseByDistance.inFrameReferenceOf(robotPose).getTranslation().norm();

            if (lookaheadDistance > kMinLookaheadDistance) {
                return lookaheadPoseByDistance;
            }
        }

        // Extend the trajectory
        double remaining =
            kMinLookaheadDistance - (lookaheadPoseByDistance.inFrameReferenceOf(robotPose).getTranslation().norm());
        return lookaheadPoseByDistance.transformBy(
                new Pose2d(new Translation2d(remaining * (iterator.getTrajectory().isReversed() ? -1 : 1), 0.0), new Rotation2d())
        );
    }

}