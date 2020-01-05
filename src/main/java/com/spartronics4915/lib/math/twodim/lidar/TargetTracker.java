package com.spartronics4915.lib.math.twodim.lidar;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.util.Units;

import edu.wpi.first.wpilibj.Timer;

public class TargetTracker {
    private static class Target {
        /** Meters */
        public final Translation2d translation;
        /** Seconds */
        public final double firstSeen;
        /** Seconds */
        public final double lastSeen;
        public final double confidence;

        public Target(Translation2d translation, double confidencePercentile) {
            this.translation = translation;
            this.firstSeen = Timer.getFPGATimestamp();
            this.lastSeen = Timer.getFPGATimestamp();
            this.confidence = confidencePercentile;
        }

        public Target(Target previous, Translation2d newTranslation, double confidencePercentile) {
            if (confidencePercentile == 1) {
                confidencePercentile *= kFirstPlaceAdvantageMultiplier;
            }

            this.translation = newTranslation;
            this.firstSeen = previous.firstSeen;
            this.lastSeen = Timer.getFPGATimestamp();
            this.confidence = previous.confidence + confidencePercentile;
        }

        /**
         * @return Time since last sighting, in seconds.
         */
        public double getTimeSinceLastSighting() {
            return Timer.getFPGATimestamp() - this.lastSeen;
        }
    }

    private static final double kMaxTargetMovementMeters = Units.inchesToMeters(8);
    private static final double kMaxTimeSinceLastSighting = 5;

    private static final double kFirstPlaceAdvantageMultiplier = 1.5;

    private List<Target> mTrackedTargets = new ArrayList<>();

    /**
     * @param targets List of targets, with the first target in the list being the "best".
     * @return The target with the highest computed score, or null if no targets were given.
     */
    public Translation2d update(List<Translation2d> targets) {
        List<Target> newTrackedTargets = new ArrayList<>();

        double highestScore = Double.NEGATIVE_INFINITY;
        Target bestTarget = null;
        for (int i = 0; i < targets.size(); i++) {
            var newTargetTranslation = targets.get(i);

            // Correlate with a previous target
            double lowestDistance = Double.POSITIVE_INFINITY;
            Target bestCorrelatedTarget = null;
            for (int j = 0; j < mTrackedTargets.size(); j++) {
                var oldTarget = mTrackedTargets.get(j);

                double dist = oldTarget.translation.getDistance(newTargetTranslation);
                if (dist < kMaxTargetMovementMeters && dist < lowestDistance) {
                    bestCorrelatedTarget = mTrackedTargets.remove(j);
                    lowestDistance = dist;
                }
            }

            final Target newTarget;
            double confidencePercentile = ((double) targets.size() - i) / targets.size();
            if (bestCorrelatedTarget == null) {
                // Target has not been seen before
                newTarget = new Target(newTargetTranslation, confidencePercentile);
            } else {
                newTarget = new Target(bestCorrelatedTarget, newTargetTranslation, confidencePercentile);
            }

            double score = newTarget.confidence;
            if (score > highestScore) {
                highestScore = score;
                bestTarget = newTarget;
            }

            newTrackedTargets.add(newTarget);
        }

        for (var oldTarget : mTrackedTargets) {
            if (oldTarget.getTimeSinceLastSighting() >= kMaxTimeSinceLastSighting) {
                continue;
            }

            double score = oldTarget.confidence;
            if (score > highestScore) {
                highestScore = score;
                bestTarget = oldTarget;
            }

            newTrackedTargets.add(oldTarget);
        }

        if (bestTarget == null) {
            return null;
        }

        mTrackedTargets = newTrackedTargets;

        return highestScore > 0 ? bestTarget.translation : null;
    }

}