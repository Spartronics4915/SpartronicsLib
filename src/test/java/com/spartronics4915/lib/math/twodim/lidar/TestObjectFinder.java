package com.spartronics4915.lib.math.twodim.lidar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.stream.Collectors;

import com.spartronics4915.lib.hardware.sensors.rplidar.RPLidarA1;
import com.spartronics4915.lib.hardware.sensors.rplidar.RPLidarMeasurement;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;

import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;

import edu.wpi.cscore.CameraServerJNI;

public class TestObjectFinder {

    static {
        // This is so libopencv_javaVERSION.so (where version is the 3-digit opencv
        // version) gets loaded.
        CameraServerJNI.forceLoad();
    }

    private List<RPLidarMeasurement> mPointcloud = new ArrayList<>();

    private int mEdgeDetectorValue = 1;
    private int mNumVotesNeeded = 5;

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
    public void testOpenCVLoading() {
        new Mat(1, 1, 1);
    }

    @Test
    public void interactivePointcloudTest() {
        final ObjectFinder finder = new ObjectFinder(0.01);
        final double circleRadiusMeters = 0.0635;

        var pointCloudCanvas = new Canvas() {
            @Override
            public void paint(Graphics g) {
                super.paint(g);

                try {
                    synchronized (mPointcloud) {
                        List<Translation2d> pointcloud = new ArrayList<>();
                        for (RPLidarMeasurement m : mPointcloud) {
                            double x, y, angleRads = Math.toRadians(m.angle);
                            // Convert to meters (TODO: Do this in RPLidar4j instead of here)
                            y = Math.sin(angleRads) * m.distance;
                            x = Math.cos(angleRads) * m.distance;

                            drawPoint(x, y, m.quality, g);

                            pointcloud.add(new Translation2d(x, y));
                        }

                        if (pointcloud.size() <= 0) {
                            return;
                        }

                        int circleDiameterCentimeters = (int) Math.round(circleRadiusMeters * 100.0 * 2.0);
                        var centers = finder.findSquares(pointcloud, new Translation2d(), 0.2032-0.01, mNumVotesNeeded, 1, 0.3);
                        // var centers = finder.findCircles(pointcloud, circleRadiusMeters, mEdgeDetectorValue, mNumVotesNeeded);
                        for (Translation2d center : centers) {
                            // System.out.println(center);
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

            private void drawPoint(double x, double y, int quality, Graphics g) {
                g.setColor(new Color(255 - quality, 0, 0));
                g.drawOval(toScreenCoordinates(x, true), toScreenCoordinates(y, false), 1, 1);
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

        RPLidarA1 lidar = new RPLidarA1("/dev/ttyUSB0", (List<RPLidarMeasurement> pointcloud) -> {
            synchronized (mPointcloud) {
                mPointcloud = pointcloud;
            }
        });
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