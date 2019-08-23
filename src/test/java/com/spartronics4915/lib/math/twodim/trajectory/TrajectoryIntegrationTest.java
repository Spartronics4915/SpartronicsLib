package com.spartronics4915.lib.math.twodim.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;

import com.spartronics4915.lib.math.twodim.control.FeedForwardTracker;
import com.spartronics4915.lib.math.twodim.control.RamseteTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.physics.DCMotorTransmission;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedState;
import com.spartronics4915.lib.math.twodim.trajectory.types.Trajectory.TrajectorySamplePoint;
import com.spartronics4915.lib.util.Units;

import org.junit.jupiter.api.Test;

public class TrajectoryIntegrationTest {

    @Test
    public void testPathFollower() {
        final double kDriveTrackScrubFactor = 1.063; // determine with auto mode
        final double kDriveTrackWidth = Units.inchesToMeters(23.75 * kDriveTrackScrubFactor);
        final double kDriveWheelDiameter = Units.inchesToMeters(6);
        final double kDriveWheelRadius = kDriveWheelDiameter / 2.0;

        final double kDriveRightVIntercept = 0.7714; // V
        final double kDriveRightKv = 0.1920; // V per rad/s
        final double kDriveRightKa = 0.0533; // V per rad/s^2

        final double kDriveLeftVIntercept = 0.7939; // V
        final double kDriveLeftKv = 0.1849; // V per rad/s
        final double kDriveLeftKa = 0.0350; // V per rad/s^2

        final double kRobotLinearInertia = 27.93; // kg (robot's mass)
        final double kRobotAngularInertia = 1.7419; // kg m^2 (use the moi auto mode)
        final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

        final DifferentialDrive kDifferentialDrive = new DifferentialDrive(
            kRobotLinearInertia, kRobotAngularInertia, kRobotAngularDrag,
            kDriveWheelRadius, kDriveTrackWidth,
            new DCMotorTransmission(
                kDriveWheelRadius, kRobotLinearInertia, kDriveLeftVIntercept, kDriveLeftKv, kDriveLeftKa
            ),
            new DCMotorTransmission(
                kDriveWheelRadius, kRobotLinearInertia, kDriveRightVIntercept, kDriveRightKv, kDriveRightKa
            )
        );
        
        var traj = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(
            Arrays.asList(new Pose2d(), new Pose2d(1, 0, new Rotation2d())),
            Arrays.asList(new DifferentialDriveDynamicsConstraint(kDifferentialDrive, 10.0)),
            0.0, 0.0, 3.65, 1.83, false, true
        );

        var tracker = new FeedForwardTracker();//new RamseteTracker(2.0, 0.7);
        tracker.reset(traj);

        var iterator = new TimedTrajectory.TimedIterator<>(traj);
        while (!tracker.isFinished()) {
            tracker.getReferencePoint();
            var out = tracker.nextState(iterator.getCurrentSample().state.state.getPose(), iterator.getProgress());

            System.out.println(out.linearVelocity);

            iterator.advance(0.1);
        }
    }

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
            // XXX: Actually, dkds doesn't match... Numerical stability?
            // final DecimalFormat fmt = new DecimalFormat("#0.000");
            // System.out.println(
            //     new Pose2dWithCurvature(
            //         new Translation2d(
            //             Units.metersToInches(state.state.getTranslation().x()), Units.metersToInches(state.state.getTranslation().y())
            //         ),
            //         state.state.getRotation(),
            //         // Units are inverse
            //         Units.inchesToMeters(state.state.getCurvature()),
            //         Units.inchesToMeters(state.state.getDCurvatureDs())
            //     ).toString() +
            //         ", t: " + fmt.format(state.time) +
            //         ", v: " + fmt.format(Units.metersToInches(state.velocity)) +
            //         ", a: " + fmt.format(Units.metersToInches(state.acceleration))
            // );

            System.out.println(state.velocity + "," + state.acceleration);
        }
    }

}