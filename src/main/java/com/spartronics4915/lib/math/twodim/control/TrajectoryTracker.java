package com.spartronics4915.lib.math.twodim.control;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedIterator;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedState;
import com.spartronics4915.lib.math.twodim.trajectory.types.Trajectory.TrajectorySamplePoint;
import com.spartronics4915.lib.util.DeltaTime;

public abstract class TrajectoryTracker {
        private TimedIterator<Pose2dWithCurvature> mTrajectoryIterator = null;
        private final DeltaTime mDeltaTimeController = new DeltaTime();
        private TrajectoryTrackerVelocityOutput mPreviousVelocity = null;

        public TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> getReferencePoint() {
                return mTrajectoryIterator.getCurrentSample();
        }

        public boolean isFinished() {
                return mTrajectoryIterator == null ? true : mTrajectoryIterator.isDone();
        }

        public void reset(TimedTrajectory<Pose2dWithCurvature> newTrajectory) {
                mTrajectoryIterator = new TimedIterator<>(newTrajectory);
                mDeltaTimeController.reset();
                mPreviousVelocity = null;
        }

        public TrajectoryTrackerOutput nextState(
                Pose2d currentRobotPoseMeters,
                double currentTimeSeconds
        ) {
                var iterator = mTrajectoryIterator;
                if (iterator == null) {
                        throw new RuntimeException("You can't get the next state from the TrajectoryTracker without a trajectory! Call TrajectoryTracker::reset first.");
                }
                double deltaTime = mDeltaTimeController.updateTime(currentTimeSeconds);
                iterator.advance(deltaTime);

                TrajectoryTrackerVelocityOutput velocity =
                        calculateState(iterator, currentRobotPoseMeters);
                
                if (mPreviousVelocity == null || deltaTime <= 0) {
                        // There should be no acceleration initially
                        return new TrajectoryTrackerOutput(
                                velocity.linearVelocity, 0.0, velocity.angularVelocity, 0.0
                        );
                } else {
                        return new TrajectoryTrackerOutput(
                                velocity.linearVelocity,
                                (velocity.linearVelocity - mPreviousVelocity.linearVelocity) / deltaTime,
                                velocity.angularVelocity,
                                (velocity.angularVelocity - mPreviousVelocity.angularVelocity) / deltaTime
                        );
                }
        }

        protected abstract TrajectoryTrackerVelocityOutput calculateState(
                TimedIterator<Pose2dWithCurvature> iterator,
                Pose2d currentRobotPoseMeters
        );

        protected static class TrajectoryTrackerVelocityOutput {
                /** Meters/sec */
                public final double linearVelocity;
                /** Rads/sec */
                public final double angularVelocity;

                public TrajectoryTrackerVelocityOutput(
                        double linearVelocityMetersSecond,
                        double angularVelocityRadsSecond
                ) {
                        this.linearVelocity = linearVelocityMetersSecond;
                        this.angularVelocity = angularVelocityRadsSecond;
                }
        }

        public static class TrajectoryTrackerOutput {
                /** Meters/sec */
                public final double linearVelocity;
                /** Meters/sec^2 */
                public final double linearAcceleration;
                /** Rads/sec */
                public final double angularVelocity;
                /** Rads/sec^2 */
                public final double angularAcceleration;

                public TrajectoryTrackerOutput(
                        double linearVelocityMetersSecond,
                        double linearAccelerationMetersSecondSq,
                        double angularVelocityRadsSecond,
                        double angularAccelerationRadsSecondSq
                ) {
                        this.linearVelocity = linearAccelerationMetersSecondSq;
                        this.linearAcceleration = linearAccelerationMetersSecondSq;
                        this.angularVelocity = angularVelocityRadsSecond;
                        this.angularAcceleration = angularAccelerationRadsSecondSq;
                }

                public DifferentialDrive.ChassisState getDifferentialDriveAcceleration() {
                        return new DifferentialDrive.ChassisState(linearAcceleration, angularAcceleration);
                }

                public DifferentialDrive.ChassisState getDifferentialDriveVelocity() {
                        return new DifferentialDrive.ChassisState(linearVelocity, angularVelocity);
                }
        }
}