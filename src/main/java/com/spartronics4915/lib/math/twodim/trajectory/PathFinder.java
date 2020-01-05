package com.spartronics4915.lib.math.twodim.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.spartronics4915.lib.math.Util;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rectangle2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.util.Units;

import com.spartronics4915.lib.math.twodim.trajectory.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class PathFinder {

    private static class Node {
        public final Vector2D point;
        /** Meters */
        public double dist;
        public Node prev;

        public Node(Vector2D point, double distMeters, Node prev) {
            this.point = point;
            this.dist = distMeters;
            this.prev = prev;
        }
    }

    private static final Rectangle2d kFieldRectangle = new Rectangle2d(new Translation2d(),
            new Translation2d(Units.feetToMeters(54), Units.feetToMeters(27)));

    /** Meters */
    private final double mRobotSize;
    /** Meters */
    private final double mRobotSizeCorner;
    /** Meters */
    private final Vector2D mRobotCornerTopRight, mRobotCornerTopLeft, mRobotCornerBottomLeft, mRobotCornerBottomRight;
    /** Meters */
    private final Translation2d mRobotTopLeftOffset, mRobotBottomRightOffset;
    /** Meters */
    private final Rectangle2d mFieldRectangleWithOffset;
    private final List<Rectangle2d> mGlobalRestrictedAreas;

    public PathFinder(double robotSizeMeters, Rectangle2d... globalRestrictedAreas) {
        mRobotSize = robotSizeMeters;
        mGlobalRestrictedAreas = Arrays.asList(globalRestrictedAreas);

        mRobotSizeCorner = Math.sqrt(Math.pow(mRobotSize, 2.0) / 2.0);
        mRobotCornerTopLeft = new Vector2D(-mRobotSizeCorner, mRobotSizeCorner);
        mRobotCornerTopRight = new Vector2D(mRobotSizeCorner, mRobotSizeCorner);
        mRobotCornerBottomLeft = new Vector2D(-mRobotSizeCorner, -mRobotSizeCorner);
        mRobotCornerBottomRight = new Vector2D(mRobotSizeCorner, -mRobotSizeCorner);
        mRobotTopLeftOffset = new Translation2d(-mRobotSize / 2.0, mRobotSize / 2.0);
        mRobotBottomRightOffset = new Translation2d(mRobotSize / 2.0, -mRobotSize / 2.0);
        mFieldRectangleWithOffset = new Rectangle2d(
                kFieldRectangle.getTopLeft().translateBy(mRobotTopLeftOffset.inverse()),
                kFieldRectangle.getBottomRight().translateBy(mRobotBottomRightOffset.inverse()));
    }

    public List<Translation2d> findInteriorWaypoints(Pose2d start, Pose2d end,
            Rectangle2d... additionalRestrictedAreas) {
        var waypoints = findWaypoints(toVector(start.getTranslation()), toVector(end.getTranslation()),
                additionalRestrictedAreas);

        List<Vector2D> ret;
        if (waypoints == null) {
            return null;
        } else if (waypoints.size() < 3) {
            ret = new ArrayList<>();
        } else {
            ret = waypoints.subList(1, waypoints.size() - 2);
        }
        return ret.stream().map((it) -> toTranslation(it)).collect(Collectors.toList());
    }

    public List<Vector2D> findWaypoints(Vector2D start, Vector2D end, Rectangle2d... additionalRestrictedAreas) {
        var effectiveRestrictedAreas = new HashSet<>(mGlobalRestrictedAreas);
        effectiveRestrictedAreas.addAll(Arrays.asList(additionalRestrictedAreas));

        var worldNodes = createNodes(effectiveRestrictedAreas);
        worldNodes.addAll(Set.of(start, end));

        return optimize(start, end, worldNodes, effectiveRestrictedAreas);
    }

    private List<Vector2D> optimize(Vector2D source, Vector2D target, Set<Vector2D> points,
            Set<Rectangle2d> effectiveRestrictedAreas) {
        final var Q = points.stream().map((it) -> new Node(it, Double.POSITIVE_INFINITY, null))
                .collect(Collectors.toSet());
        for (var node : Q) {
            if (node.point.equals(source)) {
                node.dist = 0.0;
                break;
            }
        }

        while (!Q.isEmpty()) {
            var u = Q.stream().min(Comparator.comparing((n) -> n.dist)).orElseThrow();
            Q.remove(u);
            if (u.point.equals(target)) {
                var S = new ArrayList<Vector2D>();
                var c = u;
                while (c != null) {
                    S.add(0, c.point);
                    c = c.prev;
                }
                return S;
            }

            var robotRectangle = toRobotRectangle(u.point);
            for (var v : Q) {
                var toTranslation = toTranslation(v.point.subtract(u.point));
                if (effectiveRestrictedAreas.stream()
                        .noneMatch((it) -> it.doesCollide(robotRectangle, toTranslation))) {
                    var alt = u.dist + u.point.distance(v.point);
                    if (alt < v.dist) {
                        v.dist = alt;
                        v.prev = u;
                    }
                }
            }
        }

        return null;
    }

    private Set<Vector2D> createNodes(Set<Rectangle2d> effectiveRestrictedAreas) {
        Set<Rectangle2d> effectiveRestrictedAreasWithOffsets = effectiveRestrictedAreas.stream()
                .map((rect) -> new Rectangle2d(rect.getTopLeft().translateBy(mRobotTopLeftOffset),
                        rect.getBottomRight().translateBy(mRobotBottomRightOffset)))
                .collect(Collectors.toSet());
        System.out.println(effectiveRestrictedAreasWithOffsets.size());
        var result = Pair.combinationPairs(createLines(effectiveRestrictedAreasWithOffsets)).stream()
                .map((it) -> it.first.intersection(it.second)).filter((it) -> it != null);
        System.out.println(result.collect(Collectors.toList()).size());
        var restrictedCorners = effectiveRestrictedAreas.stream()
                .flatMap((rect) -> Set.of(toVector(rect.getTopLeft()).add(mRobotCornerTopLeft),
                        toVector(rect.getTopRight()).add(mRobotCornerTopRight),
                        toVector(rect.getBottomLeft()).add(mRobotCornerBottomLeft),
                        toVector(rect.getBottomRight()).add(mRobotCornerBottomRight)).stream());

        return Stream.concat(result, restrictedCorners).filter((point) -> {
            var translation = toTranslation(point);
            return (mFieldRectangleWithOffset.contains(translation)
                    && !effectiveRestrictedAreas.stream().anyMatch((it) -> it.contains(translation)));
        }).collect(Collectors.toSet());
    }

    private Set<Line> createLines(Set<Rectangle2d> effectiveRestrictedAreas) {
        var restrictedAreas = new HashSet<>(effectiveRestrictedAreas);
        restrictedAreas.add(mFieldRectangleWithOffset);

        class LineTriple {
            public final Vector2D vecOne, vecTwo;
            public final Line line;

            public LineTriple(Translation2d first, Translation2d second) {
                this.vecOne = toVector(first);
                this.vecTwo = toVector(second);
                this.line = new Line(this.vecOne, this.vecTwo, Util.kEpsilon);
            }

            @Override
            public int hashCode() {
                return Objects.hash(vecOne, vecTwo);
            }

            @Override
            public String toString() {
                return "{" + vecOne + "; " + vecTwo + "}";
            }

            @Override
            public boolean equals(Object o) {
                if (this == o)
                    return true;
                if (o == null)
                    return false;
                if (!(o instanceof LineTriple))
                    return false;
                // If the vectors are the same then the lines are the same too
                // The Apache commons line class doesn't have an equals method
                return this.vecOne.equals(((LineTriple) o).vecOne) && this.vecTwo.equals(((LineTriple) o).vecTwo);
            }
        }

        var restrictedWallLines = restrictedAreas.stream()
                .flatMap((rect) -> Set.of(new LineTriple(rect.getTopLeft(), rect.getTopRight()),
                        new LineTriple(rect.getTopLeft(), rect.getBottomLeft()),
                        new LineTriple(rect.getBottomRight(), rect.getBottomLeft()),
                        new LineTriple(rect.getBottomRight(), rect.getTopRight())).stream())
                .collect(Collectors.toSet());
        System.out.println(restrictedWallLines.size());
        System.out.println(Pair.combinationPairs(restrictedWallLines));

        Set<Line> lines = new HashSet<>();
        for (var comboPair : Pair.combinationPairs(restrictedWallLines)) {
            if (!comboPair.first.line.isParallelTo(comboPair.second.line)
                    || comboPair.first.line.getOffset(comboPair.second.line) < mRobotSize / 2.0) {
                continue;
            }
            lines.add(new Line(comboPair.first.vecOne.add(comboPair.second.vecOne).scalarMultiply(0.5),
                    comboPair.first.vecTwo.add(comboPair.second.vecTwo).scalarMultiply(0.5), Util.kEpsilon));
        }
        System.out.println("lines " + lines.size());
        return lines;
    }

    private static final class Pair<T> {
        private final T first;
        private final T second;

        public Pair(T first, T second) {
            this.first = first;
            this.second = second;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o)
                return true;
            if (o == null)
                return false;
            if (!(o instanceof Pair<?>))
                return false;
            return first.equals(((Pair<T>) o).first) && second.equals(((Pair<T>) o).second);
        }

        @Override
        public int hashCode() {
            return Objects.hash(first, second);
        }

        public static <T> Set<Pair<T>> combinationPairs(Set<T> set) {
            var result = new HashSet<Pair<T>>();
            for (final var p1 : set) {
                for (final var p2 : set) {
                    if (p1.equals(p2) || result.contains(new Pair<T>(p2, p1)))
                        continue;
                    result.add(new Pair<T>(p1, p2));
                }
            }
            return result;
        }
    }

    private Rectangle2d toRobotRectangle(Vector2D point) {
        return new Rectangle2d((point.getX() - mRobotSize / 3), (point.getY() - mRobotSize / 3), (mRobotSize / 3 * 2),
                (mRobotSize / 3 * 2));
    }

    private Translation2d toTranslation(Vector2D vec) {
        return new Translation2d(vec.getX(), vec.getY());
    }

    private Vector2D toVector(Translation2d tran) {
        return new Vector2D(tran.getX(), tran.getY());
    }
}