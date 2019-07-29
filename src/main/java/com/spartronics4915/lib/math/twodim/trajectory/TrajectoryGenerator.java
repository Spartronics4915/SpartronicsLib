package com.spartronics4915.lib.math.twodim.trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.spline.QuinticHermiteSpline;
import com.spartronics4915.lib.math.twodim.spline.SplineGenerator;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.IndexedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;

public class TrajectoryGenerator {

    public static final TrajectoryGenerator defaultTrajectoryGenerator =
        new TrajectoryGenerator(0.0508, 0.00653, Rotation2d.fromDegrees(5));

    /** Meters */
    private final double kMaxDx, kMaxDy;
    private final Rotation2d kMaxDTheta;

    /**
     * Instantiates a trajectory generator with a couple spline parameterization
     * knobs that give coordinate deltas (in the field frame) which control how
     * frequently we parameterize a point on the spline.
     */
    public TrajectoryGenerator(double maxDxMeters, double maxDyMeters, Rotation2d maxDTheta) {
        kMaxDx = maxDxMeters;
        kMaxDy = maxDyMeters;
        kMaxDTheta = maxDTheta;
    }

    public TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
            List<TimingConstraint<Pose2dWithCurvature>> constraints, double startVelocityMetersPerSec,
            double endVelocityMetersPerSec, double maxVelocityMetersPerSec, double maxAccelerationMeterPerSecSq,
            boolean reversed, boolean optimizeSplines) {
        Pose2d flipTransform = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        // Make theta normal for trajectory generation if path is trajectoryReversed.
        List<Pose2d> newWaypoints = waypoints.stream()
                .map((point) -> reversed ? point.transformBy(flipTransform) : point).collect(Collectors.toList());

        var trajectory = trajectoryFromSplineWaypoints(newWaypoints, optimizeSplines);

        // After trajectory generation, flip theta back so it's relative to the field.
        // Also fix curvature.
        // Derivative of curvature should stay the same because the change in curvature
        // will be the same.
        if (reversed) {
            trajectory = new IndexedTrajectory<>(trajectory.stream().map((state) -> new Pose2dWithCurvature(
                state.getPose().transformBy(flipTransform),
                -state.getCurvature(),
                state.getDCurvatureDs()
            )).collect(Collectors.toList()));
        }

        return null;
    }

    private IndexedTrajectory<Pose2dWithCurvature> trajectoryFromSplineWaypoints(
        List<Pose2d> waypoints,
        boolean optimizeSplines
    ) {
        List<QuinticHermiteSpline> splines = new ArrayList<>();

        var iter = waypoints.listIterator();
        while (iter.hasNext()) {
            splines.add(new QuinticHermiteSpline(iter.next(), iter.next()));
            iter.previous(); // Send the iterator back by 1 (we're basically doing a peek)
        }

        if (optimizeSplines)
            QuinticHermiteSpline.optimizeSpline(splines);
        
        return new IndexedTrajectory<Pose2dWithCurvature>(
            SplineGenerator.parameterizeSplines(splines, kMaxDx, kMaxDy, kMaxDTheta.getRadians())
        );
    }

    
}