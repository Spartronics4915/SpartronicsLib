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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
import com.spartronics4915.lib.util.Units;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;

import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.first.wpilibj.Timer;

public class TestObjectFinder {

    private List<Translation2d> mPointcloud = new ArrayList<>();
    private long mLastResetTime = 0;

    private int mEdgeDetectorValue = 1;
    private int mNumVotesNeeded = 5;

    private final TargetTracker mTargetTracker = new TargetTracker();

    @Tag("hardwareDependant")
    @Test
    public void interactivePointcloudTest() {
        final ObjectFinder finder = new ObjectFinder(0.01);
        final double circleRadiusMeters = Units.inchesToMeters(3.5);

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
                        var center = mTargetTracker.update(centers);
                        if (center == null) {
                            System.out.println("No target found");
                            return;
                        }
                        // for (Translation2d center : centers) {
                            System.out.println(center);
                            g.setColor(Color.BLUE);
                            g.drawOval(toScreenCoordinates(center.getX() - circleRadiusMeters, true), toScreenCoordinates(center.getY() - circleRadiusMeters, false), circleDiameterCentimeters, circleDiameterCentimeters);
                            // break;
                        // }

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
                g.drawOval(toScreenCoordinates(point.getX(), true), toScreenCoordinates(point.getY(), false), 1, 1);
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
                System.out.println(k.getKeyChar());
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
        }, new RobotStateMap(), new Transform2d(new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-14)), Rotation2d.fromDegrees(180)));
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