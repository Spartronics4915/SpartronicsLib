package com.spartronics4915.lib.math.twodim.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.spartronics4915.lib.math.Util;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.spline.QuinticHermiteSpline;
import com.spartronics4915.lib.math.twodim.spline.Spline;
import com.spartronics4915.lib.math.twodim.spline.SplineHelper;
import com.spartronics4915.lib.math.twodim.spline.SplineParameterizer;
import com.spartronics4915.lib.math.twodim.spline.Spline.ControlVector;
import com.spartronics4915.lib.math.twodim.spline.SplineParameterizer.MalformedSplineException;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.DistancedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.IndexedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.State;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedState;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Units;

public class TrajectoryGenerator {

    public static final TrajectoryGenerator defaultTrajectoryGenerator = new TrajectoryGenerator(
            Units.inchesToMeters(2), Units.inchesToMeters(0.2), Rotation2d.fromDegrees(5));
    private static final TimedTrajectory<Pose2dWithCurvature> kDoNothingTrajectory = new TimedTrajectory<Pose2dWithCurvature>(
            Arrays.asList(new TimedState<Pose2dWithCurvature>(new Pose2dWithCurvature(), 0, 0, 0)), false);

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

    public TimedTrajectory<Pose2dWithCurvature> generateTrajectory(Pose2d start, List<Translation2d> interiorWaypoints,
            Pose2d end, List<TimingConstraint<Pose2dWithCurvature>> constraints, double startVelocityMetersPerSec,
            double endVelocityMetersPerSec, double maxVelocityMetersPerSec, double maxAccelerationMeterPerSecSq,
            boolean reversed) {
        var controlVectors = SplineHelper.getCubicControlVectorsFromWaypoints(start,
                interiorWaypoints.toArray(new Translation2d[0]), end);
        final var flipTransform = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        // Clone the control vectors.
        var newInitial = new Spline.ControlVector(controlVectors[0].x, controlVectors[0].y);
        var newEnd = new Spline.ControlVector(controlVectors[1].x, controlVectors[1].y);

        // Change the orientation if reversed.
        if (reversed) {
            newInitial.x[1] *= -1;
            newInitial.y[1] *= -1;
            newEnd.x[1] *= -1;
            newEnd.y[1] *= -1;
        }

        // Get the spline points
        List<Pose2dWithCurvature> points;
        try {
            points = splinePointsFromSplines(SplineHelper.getCubicSplinesFromControlVectors(newInitial,
                    interiorWaypoints.toArray(new Translation2d[0]), newEnd));
        } catch (MalformedSplineException e) {
            Logger.exception(e);
            return kDoNothingTrajectory;
        }

        // Change the points back to their original orientation.
        var trajectory = new IndexedTrajectory<>(points);
        if (reversed) {
            trajectory = new IndexedTrajectory<>(trajectory.stream()
                    .map((state) -> new Pose2dWithCurvature(state.getPose().transformBy(flipTransform),
                            -state.getCurvature(), state.getDCurvatureDs()))
                    .collect(Collectors.toList()));
        }

        return timeParameterizeTrajectory(new DistancedTrajectory<Pose2dWithCurvature>(trajectory), constraints,
                startVelocityMetersPerSec, endVelocityMetersPerSec, maxVelocityMetersPerSec,
                Math.abs(maxAccelerationMeterPerSecSq), kMaxDx, reversed);
    }

    public TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
            List<TimingConstraint<Pose2dWithCurvature>> constraints, double startVelocityMetersPerSec,
            double endVelocityMetersPerSec, double maxVelocityMetersPerSec, double maxAccelerationMeterPerSecSq,
            boolean reversed) {
        Pose2d flipTransform = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        // Make theta normal for trajectory generation if path is trajectoryReversed.
        List<Pose2d> newWaypoints = waypoints.stream()
                .map((point) -> reversed ? point.transformBy(flipTransform) : point).collect(Collectors.toList());

        var controlVectors = SplineHelper.getQuinticControlVectorsFromWaypoints(newWaypoints);

        List<Pose2dWithCurvature> points;
        try {
            points = splinePointsFromSplines(SplineHelper
                    .getQuinticSplinesFromControlVectors(controlVectors.toArray(new Spline.ControlVector[] {})));
        } catch (Exception e) {
            Logger.exception(e);
            return kDoNothingTrajectory;
        }
        var trajectory = new IndexedTrajectory<>(points);

        // After trajectory generation, flip theta back so it's relative to the field.
        // Also fix curvature.
        // Derivative of curvature should stay the same because the change in curvature
        // will be the same.
        if (reversed) {
            trajectory = new IndexedTrajectory<>(trajectory.stream()
                    .map((state) -> new Pose2dWithCurvature(state.getPose().transformBy(flipTransform),
                            -state.getCurvature(), state.getDCurvatureDs()))
                    .collect(Collectors.toList()));
        }

        return timeParameterizeTrajectory(new DistancedTrajectory<Pose2dWithCurvature>(trajectory), constraints,
                startVelocityMetersPerSec, endVelocityMetersPerSec, maxVelocityMetersPerSec,
                Math.abs(maxAccelerationMeterPerSecSq), kMaxDx, reversed);
    }

    public static List<Pose2dWithCurvature> splinePointsFromSplines(Spline[] splines) {
        // Create the vector of spline points.
        var splinePoints = new ArrayList<Pose2dWithCurvature>();

        // Add the first point to the vector.
        splinePoints.add(splines[0].getPoint(0.0));

        // Iterate through the vector and parameterize each spline, adding the
        // parameterized points to the final vector.
        for (final var spline : splines) {
            var points = SplineParameterizer.parameterize(spline);

            // Append the array of poses to the vector. We are removing the first
            // point because it's a duplicate of the last point from the previous
            // spline.
            splinePoints.addAll(points.subList(1, points.size()));
        }
        return splinePoints;
    }

    private <S extends State<S>> TimedTrajectory<S> timeParameterizeTrajectory(
            /*
             * Note that we don't follow the units naming convention here. This is usually
             * bad, but this is a private method and we're trying to be less verbose.
             * 
             * All units are meters (step size), meters/sec (velocity) or meters/sec^2
             * (acceleration).
             */
            DistancedTrajectory<S> distancedTrajectory, List<TimingConstraint<S>> constraints, double startVelocity,
            double endVelocity, double maxVelocity, double maxAcceleration, double stepSize, boolean reversed) {
        class ConstrainedState {
            public S state;
            /** Meters */
            public double distance = 0;
            /** Meters/sec */
            public double maxVelocity = 0;
            /** Meters/sec^2 */
            public double minAcceleration = 0, maxAcceleration = 0;
        }

        var accelLimiter = new Object() {
            public void enforceAccelerationLimits(boolean reversed, List<TimingConstraint<S>> constraints,
                    ConstrainedState constrainedState) {
                for (TimingConstraint<S> constraint : constraints) {
                    var minMaxAccel = constraint.getMinMaxAcceleration(constrainedState.state,
                            (reversed ? -1 : 1) * constrainedState.maxVelocity);
                    if (!minMaxAccel.valid) {
                        throw new RuntimeException(
                                "Constraint returned an invalid minMaxAccel for state " + constrainedState.state);
                    }

                    constrainedState.minAcceleration = Math.max(constrainedState.minAcceleration,
                            reversed ? -minMaxAccel.maxAcceleration : minMaxAccel.minAcceleration);

                    constrainedState.maxAcceleration = Math.min(constrainedState.maxAcceleration,
                            reversed ? -minMaxAccel.minAcceleration : minMaxAccel.maxAcceleration);
                }
            }
        };

        int distanceViewSteps = (int) Math.ceil(
                (distancedTrajectory.getLastInterpolant() - distancedTrajectory.getFirstInterpolant()) / stepSize + 1);

        List<S> states = new ArrayList<>();
        for (int i = 0; i < distanceViewSteps; i++) {
            S state = distancedTrajectory.sample(Util.limit(i * stepSize + distancedTrajectory.getFirstInterpolant(),
                    distancedTrajectory.getFirstInterpolant(), distancedTrajectory.getLastInterpolant())).state;
            states.add(state);
        }

        List<ConstrainedState> constrainedStates = new ArrayList<>(states.size());
        double epsilon = 1e-6;

        // Note that the forwards and backwards passes (i.e. everything below this
        // point) are looking to find one thing, and one thing only, for each state: max
        // acheiveable velocity given the constraints and the other adjacent states. All
        // the other mins and maxes, and all the acceleration info is just bookkeeping
        // to get to that point.

        // Forward pass. We look at pairs of consecutive states, where the start state
        // has already been velocity parameterized (though we may adjust the velocity
        // downwards during the backwards pass). We wish to find an acceleration that is
        // admissible at both the start and end state, as well as an admissible end
        // velocity. If there is no admissible end velocity or acceleration, we set the
        // end velocity to the state's maximum allowed velocity and will repair the
        // acceleration during the backward pass (by slowing down the predecessor).

        var predecessor = new ConstrainedState();
        predecessor.state = states.get(0);
        predecessor.distance = 0.0;
        predecessor.maxVelocity = startVelocity;
        predecessor.minAcceleration = -maxAcceleration;
        predecessor.maxAcceleration = maxAcceleration;

        for (S s : states) {
            var constrainedState = new ConstrainedState();
            constrainedState.state = s;
            /** Meters */
            double ds = constrainedState.state.distance(predecessor.state);
            constrainedState.distance = ds + predecessor.distance;
            constrainedStates.add(constrainedState);

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global acceleration
                // limit.
                // vf = sqrt(vi^2 + 2*a*d)
                // (This equation finds a final velocity given an initial velocity,
                // acceleration, and distance)
                constrainedState.maxVelocity = Math.min(maxVelocity,
                        Math.sqrt(Math.pow(predecessor.maxVelocity, 2) + 2 * predecessor.maxAcceleration * ds));
                if (Double.isNaN(constrainedState.maxVelocity)) {
                    throw new RuntimeException("Max velocicity is NaN");
                }
                // Enforce global max absolute acceleration
                constrainedState.minAcceleration = -maxAcceleration;
                constrainedState.maxAcceleration = maxAcceleration;

                // At this point, the state is fully constructed, but no constraints have been
                // applied aside from predecessor state max accel.

                // Enforce all velocity constraints
                for (var constraint : constraints) {
                    constrainedState.maxVelocity = Math.min(constrainedState.maxVelocity,
                            constraint.getMaxVelocity(constrainedState.state));
                }
                if (constrainedState.maxVelocity < 0.0) {
                    // This should never happen if constraints are well-behaved
                    throw new RuntimeException("constrainedState max velocity cannot be negative");
                }

                // Now enforce all acceleration constraints
                accelLimiter.enforceAccelerationLimits(reversed, constraints, constrainedState);
                if (constrainedState.minAcceleration > constrainedState.maxAcceleration) {
                    // This should never happen if constraints are well-behaved
                    throw new RuntimeException(
                            "constrainedState min acceleration cannot be greater than max acceleration");
                }

                if (ds < epsilon) {
                    break;
                }

                double actualAcceleration = (Math.pow(constrainedState.maxVelocity, 2)
                        - Math.pow(predecessor.maxVelocity, 2)) / (2 * ds);
                if (constrainedState.maxAcceleration < actualAcceleration - epsilon) {
                    // If the max acceleration for this constraint state is more conservative than
                    // what we had applied, we need to reduce the max accel at the predecessor state
                    // and try again.
                    predecessor.maxAcceleration = constrainedState.maxAcceleration;
                } else {
                    if (actualAcceleration > predecessor.minAcceleration + epsilon) {
                        predecessor.maxAcceleration = actualAcceleration;
                    }
                    // If actual acceleration is less than predecessor min accel, we will repair
                    // during the backward pass.
                    break;
                }
            }
            predecessor = constrainedState;
        }

        // Backwards pass
        var successor = new ConstrainedState();
        successor.state = states.get(states.size() - 1);
        successor.distance = constrainedStates.get(states.size() - 1).distance;
        successor.maxVelocity = endVelocity;
        successor.minAcceleration = -maxAcceleration;
        successor.maxAcceleration = maxAcceleration;
        for (int i = states.size() - 1; i >= 0; i--) {
            ConstrainedState constrainedState = constrainedStates.get(i);
            /** Meters */
            double ds = constrainedState.distance - successor.distance; // Will be negative

            while (true) {
                // Enforce reverse max reachable velocity limit
                // vf = sqrt(vi^2 + 2*a*d), where vi = successor
                var newMaxVelocity = Math.sqrt(Math.pow(successor.maxVelocity, 2) + 2 * successor.minAcceleration * ds);
                if (newMaxVelocity >= constrainedState.maxVelocity) {
                    // No new limits to impose
                    break;
                }
                constrainedState.maxVelocity = newMaxVelocity;
                if (Double.isNaN(constrainedState.maxVelocity)) {
                    throw new RuntimeException("constrainedState max velocity can't be NaN");
                }

                // Now check all acceleration constraints with lower max velocity
                if (constrainedState.minAcceleration > constrainedState.maxAcceleration) {
                    throw new RuntimeException(
                            "constrainedState min acceleration cannot be greater than max acceleration");
                }

                if (ds > epsilon) {
                    break;
                }
                // If the min acceleration for this constraint state is more conservative than
                // what we have applied then we need to reduce the min accel and try again
                double actualAcceleration = (Math.pow(constrainedState.maxVelocity, 2)
                        - Math.pow(successor.maxVelocity, 2)) / (2 * ds);
                if (constrainedState.minAcceleration > actualAcceleration + epsilon) {
                    successor.minAcceleration = constrainedState.minAcceleration;
                } else {
                    // This sets successor.minAcceleration to NaN on the first iteration (because ds
                    // is 0), but it doesn't matter
                    successor.minAcceleration = actualAcceleration;
                    break;
                }
            }
            successor = constrainedState;
        }

        // Integrate the constrained states forward in time to obtain the TimedStates
        List<TimedState<S>> timedStates = new ArrayList<>(states.size());
        /** Current time in seconds */
        double t = 0;
        /** Current distance in meters */
        double s = 0;
        /** Current velocity in meters/sec */
        double v = 0;
        for (int i = 0; i < states.size(); i++) {
            ConstrainedState constrainedState = constrainedStates.get(i);

            // Here we find the dt to advance t by
            // Finding the unknown, dt, requires the current state's max velocity, the last
            // state's actual velocity, and ds (delta distance traveled)
            double ds = constrainedState.distance - s;
            double accel = (Math.pow(constrainedState.maxVelocity, 2) - Math.pow(v, 2)) / (2.0 * ds);
            double dt = 0.0;
            if (i > 0) {
                timedStates.set(i - 1, new TimedState<S>(timedStates.get(i - 1), reversed ? -accel : accel));

                if (Math.abs(accel) > epsilon) {
                    dt = (constrainedState.maxVelocity - v) / accel;
                } else if (Math.abs(v) > epsilon) {
                    dt = ds / v;
                } else {
                    throw new RuntimeException(
                            "Couldn't determine a dt to integrate because robot is too still after the initial pose");
                }
            }
            t += dt;
            if (Double.isNaN(t) || Double.isInfinite(t)) {
                throw new RuntimeException();
            }

            v = constrainedState.maxVelocity;
            s = constrainedState.distance;
            timedStates.add(new TimedState<S>(constrainedState.state, t, reversed ? -v : v, reversed ? -accel : accel));
        }

        return new TimedTrajectory<S>(timedStates, reversed);
    }

}