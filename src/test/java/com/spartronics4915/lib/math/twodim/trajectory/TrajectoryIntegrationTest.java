package com.spartronics4915.lib.math.twodim.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.physics.DCMotorTransmission;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.Trajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedState;
import com.spartronics4915.lib.math.twodim.trajectory.types.Trajectory.TrajectorySamplePoint;
import com.spartronics4915.lib.util.Units;

import org.junit.jupiter.api.Test;

public class TrajectoryIntegrationTest {

    // @Test
    // public void testFollowerTrajectoryGenerator() {
    //     // Specify desired waypoints.
    //     List<Translation2d> waypoints = Arrays.asList(new Translation2d(0.0, 0.0), new Translation2d(24.0, 0.0),
    //             new Translation2d(36.0, 0.0), new Translation2d(36.0, 24.0), new Translation2d(60.0, 24.0));

    //     // Create the reference trajectory (straight line motion between waypoints).
    //     Trajectory<Translation2d> reference_trajectory = new Trajectory<>(waypoints);

    //     // Generate a smooth (continuous curvature) path to follow.
    //     IPathFollower path_follower = new PurePursuitController<Translation2d>(new DistanceView<>(reference_trajectory),
    //             /* sampling_dist */1.0, /* lookahead= */ 6.0, /* goal_tolerance= */ 0.1);
    //     Trajectory<Pose2dWithCurvature> smooth_path = TrajectoryUtil.trajectoryFromPathFollower(path_follower,
    //             Pose2dWithCurvature.identity(), /* step_size= */ 1.0, /* dcurvature_limit= */1.0);

    //     assertFalse(smooth_path.isEmpty());
    //     System.out.println(smooth_path.toCSV());

    //     // Time parameterize the path subject to our dynamic constraints.
    //     // TODO
    // }

    @Test
    public void testSplineTrajectoryGenerator() {
        // Specify desired waypoints.
        List<Pose2d> waypoints = Arrays.asList(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(Units.inchesToMeters(36.0), Units.inchesToMeters(0.0), Rotation2d.fromDegrees(0.0)),
                new Pose2d(Units.inchesToMeters(60.0), Units.inchesToMeters(100), Rotation2d.fromDegrees(0.0)),
                new Pose2d(Units.inchesToMeters(160.0), Units.inchesToMeters(100.0), Rotation2d.fromDegrees(0.0)),
                new Pose2d(Units.inchesToMeters(200.0), Units.inchesToMeters(70), Rotation2d.fromDegrees(45.0))
        );

        // Create a differential drive.
        final double kRobotMassKg = 60.0;
        final double kRobotAngularInertia = 80.0;
        final double kRobotAngularDrag = 0.0;
        final double kWheelRadius = Units.inchesToMeters(2.0);
        DCMotorTransmission transmission = new DCMotorTransmission(1.0 / 0.143,
                (kWheelRadius * kWheelRadius * kRobotMassKg / 2.0) / 0.02, 0.8);
        DifferentialDrive drive = new DifferentialDrive(kRobotMassKg, kRobotAngularInertia, kRobotAngularDrag,
                kWheelRadius, Units.inchesToMeters(26.0 / 2.0), transmission, transmission);

        // Create the constraint that the robot must be able to traverse the trajectory
        // without ever applying more
        // than 10V.
        var voltageConstraint = new DifferentialDriveDynamicsConstraint(drive, 10.0);

        // Generate the timed trajectory.
        var trajectory = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(
            waypoints, Arrays.asList(voltageConstraint), 0.0, 0.0, Units.inchesToMeters(12 * 14), 
                Units.inchesToMeters(12 * 10), false, true
        );

        for (int i = 1; i < trajectory.size(); i++) {
            final var prev = trajectory.getPoint(i - 1);
            final var next = trajectory.getPoint(i);
            final double dt = next.state.time - prev.state.time;

            assertEquals(prev.state.acceleration, (next.state.velocity - prev.state.velocity) / dt, 1E-9);
            assertEquals(next.state.velocity, prev.state.velocity + prev.state.acceleration * dt, 1E-9);
            assertEquals(next.state.distance(prev.state),
                    prev.state.velocity * dt + 0.5 * prev.state.acceleration * dt * dt, 1E-9);
        }

        // "Follow" the trajectory.
        final double kDt = 0.01;
        boolean first = true;
        var it = new TimedTrajectory.TimedIterator<>(trajectory);
        while (!it.isDone()) {
            TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample;
            if (first) {
                sample = it.getCurrentSample();
                first = false;
            } else {
                sample = it.advance(kDt);
            }
            final TimedState<Pose2dWithCurvature> state = sample.state;

            // This is designed to be exactly the same as 2019-DeepSpace output, as a comparison
            // final DecimalFormat fmt = new DecimalFormat("#0.000");
            // System.out.println(
            //     new Pose2dWithCurvature(
            //         new Translation2d(
            //             Units.metersToInches(state.state.getTranslation().x()), Units.metersToInches(state.state.getTranslation().y())
            //         ),
            //         state.state.getRotation(),
            //         state.state.getCurvature() / 0.0254,
            //         state.state.getDCurvatureDs() / 0.0254
            //     ).toString() +
            //         ", t: " + fmt.format(state.time) +
            //         ", v: " + fmt.format(Units.metersToInches(state.velocity)) +
            //         ", a: " + fmt.format(Units.metersToInches(state.acceleration))
            // );
        }
    }

}