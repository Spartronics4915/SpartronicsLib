/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.spartronics4915.lib.math.twodim.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;

import static com.spartronics4915.lib.util.Units.feetToMeters;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TrajectoryGeneratorTest {
  static TimedTrajectory<Pose2dWithCurvature> getTrajectory(List<TimingConstraint<Pose2dWithCurvature>> constraints) {
    final double maxVelocity = feetToMeters(12.0);
    final double maxAccel = feetToMeters(12);

    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(feetToMeters(1.54), feetToMeters(23.23), Rotation2d.fromDegrees(-180));
    var crossScale = new Pose2d(feetToMeters(23.7), feetToMeters(6.8), Rotation2d.fromDegrees(-160));

    var waypoints = new ArrayList<Pose2d>();
    waypoints.add(sideStart);
    waypoints.add(
        sideStart.transformBy(new Pose2d(new Translation2d(feetToMeters(-13), feetToMeters(0)), new Rotation2d())));
    waypoints.add(sideStart
        .transformBy(new Pose2d(new Translation2d(feetToMeters(-19.5), feetToMeters(5)), Rotation2d.fromDegrees(-90))));
    waypoints.add(crossScale);

    return TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(waypoints, constraints, 0, 0, maxVelocity,
        maxAccel, true);
  }

  @Test
  @SuppressWarnings("LocalVariableName")
  void testGenerationAndConstraints() {
    TimedTrajectory<Pose2dWithCurvature> trajectory = getTrajectory(new ArrayList<>());

    double duration = trajectory.getTotalTime();
    double t = 0.0;
    double dt = 0.02;

    while (t < duration) {
      var point = trajectory.sample(t);
      t += dt;
      assertAll(() -> assertTrue(Math.abs(point.state.velocity) < feetToMeters(12.0) + 0.05),
          () -> assertTrue(Math.abs(point.state.acceleration) < feetToMeters(12.0) + 0.05));
    }
  }

  @Test
  void testMalformedTrajectory() {
    var traj = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(1, 0, Rotation2d.fromDegrees(180))),
        Arrays.asList(), 0, 0, 12, 12, false);

    assertEquals(traj.size(), 1);
    assertEquals(traj.getTotalTime(), 0);
  }
}
