package com.spartronics4915.lib.math.twodim.lidar;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.lib.hardware.sensors.RPLidarA1;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.util.Units;

import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;

import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.first.wpilibj.Timer;

public class TestObjectFinder {

    private List<Translation2d> mPointcloud = new ArrayList<>();
    private long mLastResetTime = 0;

    private int mEdgeDetectorValue = 1;
    private int mNumVotesNeeded = 3;

    // @Test
    // public void testCoords() {
    //     var obj = new ObjectFinder(0.01);

    //     var a = new Translation2d(-2, 1);
    //     var b = new Translation2d(-2, 3);
    //     assertEquals(obj.getSquareCenter(a, b), new Translation2d(-4, 2));

    //     b = new Translation2d(2, 1);
    //     a = new Translation2d(2, 3);
    //     assertEquals(obj.getSquareCenter(a, b), new Translation2d(4, 2));

    //     a = new Translation2d(2, -1);
    //     b = new Translation2d(2, -3);
    //     assertEquals(obj.getSquareCenter(a, b), new Translation2d(4, -2));

    //     b = new Translation2d(-2, -1);
    //     a = new Translation2d(-2, -3);
    //     assertEquals(obj.getSquareCenter(a, b), new Translation2d(-4, -2));
    // }

    @Test
    public void testCoords() {
        var point = new Pose2d(0, 1, new Rotation2d());

        var fieldToVehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        var vehicleToLidar = new Pose2d(1, -1, Rotation2d.fromDegrees(-90));

        System.out.println(fieldToVehicle.transformBy(vehicleToLidar).transformBy(point));
    }

    @Test
    public void interactivePointcloudTest() {
        final ObjectFinder finder = new ObjectFinder(0.01);
        final double circleRadiusMeters = Units.inchesToMeters(9);

        var pointCloudCanvas = new Canvas() {
            @Override
            public void paint(Graphics g) {
                super.paint(g);

                try {
                    synchronized (mPointcloud) {
                        if (mPointcloud.size() <= 0) {
                            return;
                        }

                        mPointcloud.forEach((p) -> drawPoint(p, 255, g));

                        int circleDiameterCentimeters = (int) Math.round(circleRadiusMeters * 100.0 * 2.0);
                        // var centers = finder.findSquares(pointcloud, new Translation2d(), 0.28, mNumVotesNeeded, 3, 0.3);
                        var centers = finder.findCircles(mPointcloud, circleRadiusMeters, mNumVotesNeeded, 3);
                        for (Translation2d center : centers) {
                            System.out.println(center);
                            g.setColor(Color.BLUE);
                            g.drawOval(toScreenCoordinates(center.x() - circleRadiusMeters, true), toScreenCoordinates(center.y() - circleRadiusMeters, false), circleDiameterCentimeters, circleDiameterCentimeters);
                            break;
                        }

                        g.drawString("Edge: " + mEdgeDetectorValue + ", Votes: " + mNumVotesNeeded, 10, 10);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            // Screen coordinates correspond to centimeters
            private int toScreenCoordinates(double coordMeters, boolean isX) {
                Dimension size = this.getSize();

                coordMeters *= 100;
                coordMeters += (isX ? size.getWidth() : size.getHeight()) / 2;

                return (int) Math.round(coordMeters);
            }

            private void drawPoint(Translation2d point, int quality, Graphics g) {
                g.setColor(new Color(255 - quality, 0, 0));
                g.drawOval(toScreenCoordinates(point.x(), true), toScreenCoordinates(point.y(), false), 1, 1);
            }
        };
        pointCloudCanvas.setSize(6000, 6000);
        pointCloudCanvas.setBackground(Color.WHITE);

        var frame = new Frame();
        frame.add(pointCloudCanvas);
        frame.setSize(6000, 6000);
        frame.setVisible(true);

        frame.addKeyListener(new KeyListener() {

            @Override
            public void keyPressed(KeyEvent k) {
            }

            @Override
            public void keyReleased(KeyEvent k) {
                if (k.getKeyChar() == '-' || k.getKeyChar() == '+') {
                    double toAdd = (k.getKeyChar() == '+' ? 1 : -1);

                    if (k.isAltDown()) {
                        mEdgeDetectorValue += toAdd;
                    } else if (k.isControlDown()) {
                        mNumVotesNeeded += toAdd;
                    }
                } else if (k.getKeyChar() == 'q') {
                    System.exit(0);
                }
            }

            @Override
            public void keyTyped(KeyEvent k) {
            }

        });

        RPLidarA1 lidar = new RPLidarA1();
        lidar.setCallback((List<Translation2d> pointcloud) -> {
            synchronized (mPointcloud) {
                if (System.currentTimeMillis() - mLastResetTime > 0) {
                    mPointcloud = pointcloud;
                    mLastResetTime = System.currentTimeMillis();
                } else {
                    mPointcloud.addAll(pointcloud);
                }
            }
        }, new RobotStateMap(), new Pose2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-14), Rotation2d.fromDegrees(180)));
        lidar.start();

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            lidar.stop();
            System.out.println("Graceful shutdown complete");
        }));

        while (true) {
            try {
                Thread.sleep(500);
                synchronized (mPointcloud) {
                    pointCloudCanvas.repaint();                    
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}