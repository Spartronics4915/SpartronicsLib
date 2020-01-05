package com.spartronics4915.lib.math.twodim.trajectory;

import static com.spartronics4915.lib.util.Units.feetToMeters;
import static com.spartronics4915.lib.util.Units.inchesToMeters;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rectangle2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory.TimedState;
import com.spartronics4915.lib.math.twodim.trajectory.types.Trajectory.TrajectorySamplePoint;

import org.junit.jupiter.api.Test;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;

import java.awt.Color;
import java.awt.Font;

class PathFinderTest {
    @Test
    public void testPathFinder() {
        var robotSize = inchesToMeters(33.0);
        var pathFinder = new PathFinder(robotSize,
                new Rectangle2d(new Translation2d(inchesToMeters(140.0), inchesToMeters(85.25)),
                        new Translation2d(inchesToMeters(196.0), inchesToMeters(238.75))),
                new Rectangle2d(new Translation2d(inchesToMeters(261.47), inchesToMeters(95.25)),
                        new Translation2d(inchesToMeters(196.0), inchesToMeters(238.75))),
                new Rectangle2d(new Translation2d(inchesToMeters(196.0), inchesToMeters(85.25)),
                        new Translation2d(inchesToMeters(211.0), inchesToMeters(238.75))));

        double start = System.currentTimeMillis();

        var startPose = new Pose2d(feetToMeters(1.54), feetToMeters(23.234167), Rotation2d.fromDegrees(0));
        var endPose = new Pose2d(feetToMeters(23.7), feetToMeters(27 - 20.2), Rotation2d.fromDegrees(0));
        var interiors = pathFinder.findInteriorWaypoints(startPose, endPose,
                new Rectangle2d(new Translation2d(feetToMeters(0), feetToMeters(0)),
                        new Translation2d(feetToMeters(10), feetToMeters(10))));
        assertNotEquals(interiors, null);
        var traj = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(startPose, interiors, endPose,
                Arrays.asList(new CentripetalAccelerationConstraint(4.0)), 0, 0, feetToMeters(10), feetToMeters(4),
                false);

        double end = System.currentTimeMillis();
        System.out.println(end - start);

        List<TrajectorySamplePoint<TimedState<Pose2dWithCurvature>>> refList = new ArrayList<>();
        final var dt = 0.1;
        var t = 0.0;
        while (t < traj.getTotalTime()) {
            refList.add(traj.sample(t));
            t += dt;
        }

        var fm = new DecimalFormat("#.###").format(traj.getTotalTime());

        var chart = new XYChartBuilder().width(1800).height(1520).title(fm.toString() + " seconds.").xAxisTitle("X")
                .yAxisTitle("Y").build();

        chart.getStyler().setMarkerSize(8);
        chart.getStyler().setSeriesColors(new Color[] { Color.ORANGE, new Color(151, 60, 67) });

        chart.getStyler().setChartTitleFont(new Font("Kanit", 1, 40));
        chart.getStyler().setChartTitlePadding(15);

        chart.getStyler().setXAxisMin(1.0);
        chart.getStyler().setXAxisMax(26.0);
        chart.getStyler().setYAxisMin(1.0);
        chart.getStyler().setYAxisMax(26.0);

        chart.getStyler().setChartFontColor(Color.WHITE);
        chart.getStyler().setAxisTickLabelsColor(Color.WHITE);

        chart.getStyler().setLegendBackgroundColor(Color.GRAY);

        chart.getStyler().setPlotGridLinesVisible(true);
        chart.getStyler().setLegendVisible(true);

        chart.getStyler().setPlotGridLinesColor(Color.GRAY);
        chart.getStyler().setChartBackgroundColor(Color.DARK_GRAY);
        chart.getStyler().setPlotBackgroundColor(Color.DARK_GRAY);

        chart.addSeries("Trajectory",
                refList.stream().map((it) -> it.state.state.getTranslation().getX()).collect(Collectors.toList()),
                refList.stream().map((it) -> it.state.state.getTranslation().getY()).collect(Collectors.toList()));

        var restricted = Arrays.asList(
                new Rectangle2d(new Translation2d(feetToMeters(0), feetToMeters(0)),
                        new Translation2d(feetToMeters(10), feetToMeters(10))),
                new Rectangle2d(new Translation2d(inchesToMeters(140.0), inchesToMeters(85.25)),
                        new Translation2d(inchesToMeters(196.0), inchesToMeters(238.75))),
                new Rectangle2d(new Translation2d(inchesToMeters(261.47), inchesToMeters(95.25)),
                        new Translation2d(inchesToMeters(196.0), inchesToMeters(238.75))),
                new Rectangle2d(new Translation2d(inchesToMeters(196.0), inchesToMeters(85.25)),
                        new Translation2d(inchesToMeters(211.0), inchesToMeters(238.75))));
        chart.addSeries("Restricted Areas",
                restricted.stream()
                        .flatMap((it) -> Arrays.asList(it.getBottomLeft().getX(), it.getTopLeft().getX(), it.getBottomRight().getX(),
                                it.getTopRight().getX()).stream())
                        .collect(Collectors.toList()),
                restricted.stream()
                        .flatMap((it) -> Arrays.asList(it.getBottomLeft().getY(), it.getTopLeft().getY(),
                                it.getBottomRight().getY(), it.getTopRight().getY()).stream())
                        .collect(Collectors.toList()));

        new SwingWrapper<XYChart>(chart).displayChart();
        try {
            Thread.sleep(10000000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}