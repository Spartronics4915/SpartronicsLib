package com.spartronics4915.lib.math.twodim.lidar;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.util.Units;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CameraServerJNI;

public class ObjectFinder {

    static {
        // This is so libopencv_javaVERSION.so (where version is the 3-digit opencv
        // version) gets loaded.
        try {
            CameraServerJNI.forceLoad();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /** Side length of each binning square in meters */
    private final double mBinWidth;

    public ObjectFinder(double binWidthMeters) {
        mBinWidth = binWidthMeters;
    }

    public List<Translation2d> findCircles(List<Translation2d> pointcloud, double radiusMeters, int minVotes, int dilationSize) {
        var binned = toMat(pointcloud);
        Mat binnedPointcloud = binned.getKey();
        BinnedPoint minPoint = binned.getValue();

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(dilationSize, dilationSize));
        Imgproc.dilate(binnedPointcloud, binnedPointcloud, kernel);

        Mat results = new Mat();
        int roundedRadius = (int) Math.round(radiusMeters / mBinWidth);
        Imgproc.HoughCircles(binnedPointcloud, results, Imgproc.HOUGH_GRADIENT, 1.0, (radiusMeters * 2) / mBinWidth,
                1, minVotes, roundedRadius - 3, roundedRadius + 3);

        List<Translation2d> centers = new ArrayList<>();
        for (int i = 0; i < results.cols(); i++) {
            double[] result = results.get(0, i);
            centers.add(binnedCoordsToTranslation2d(result[0], result[1], minPoint));
        }

        return centers;
    }

    public List<Translation2d> findSquares(List<Translation2d> pointcloud, Translation2d sensorPosition, double sideLengthMeters, int minVotes, int dilationSize, double maxLineGapMeters) {
        var binned = toMat(pointcloud);
        Mat binnedPointcloud = binned.getKey();
        BinnedPoint minPoint = binned.getValue();

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(dilationSize, dilationSize));
        Imgproc.dilate(binnedPointcloud, binnedPointcloud, kernel);

        Imgcodecs.imwrite("binned.png", binnedPointcloud);

        Mat results = new Mat();
        Imgproc.HoughLinesP(binnedPointcloud, results, 1, Math.PI / 360, minVotes, sideLengthMeters / mBinWidth, maxLineGapMeters / mBinWidth);

        System.out.println(results.dump());

        List<Translation2d> centers = new ArrayList<>();
        for (int i = 0; i < results.cols(); i++) {
            double[] points = results.get(0, i);

            var pointOne = binnedCoordsToTranslation2d(points[0], points[1], minPoint);
            var pointTwo = binnedCoordsToTranslation2d(points[2], points[3], minPoint);

            if (pointOne.getDistance(pointTwo) > sideLengthMeters + Units.inchesToMeters(2)) {
                continue;
            }

            System.out.println(pointOne + ", " + pointTwo);

            centers.add(getSquareCenter(pointOne, pointTwo));
        }

        System.out.println();

        return centers;
    }

    private Translation2d binnedCoordsToTranslation2d(double x, double y, BinnedPoint minPoint) {
        x += minPoint.x();
        y += minPoint.y();

        return new Translation2d(mBinWidth * x, mBinWidth * y);
    }

    private Translation2d getSquareCenter(Translation2d a, Translation2d b) {
        final double sideLength = a.getDistance(b);

        var midpoint = a.translateBy(b).scale(0.5);

        var normalOne = a.translateBy(midpoint.inverse()).rotateBy(Rotation2d.fromDegrees(90));
        var normalTwo = a.translateBy(midpoint.inverse()).rotateBy(Rotation2d.fromDegrees(270));

        Rotation2d normalOneAngle = normalOne.getAngle(midpoint);
        Rotation2d normalTwoAngle = normalTwo.getAngle(midpoint);

        Translation2d center = (Math.abs(normalOneAngle.getRadians()) > Math.abs(normalTwoAngle.getRadians())
                ? normalTwo
                : normalOne);
        center = center.scale(1 / center.norm()).scale(sideLength).translateBy(midpoint);

        return center;
    }

    /**
     * @param pointcloud A pointcloud in field coordinates to bin.
     * @return An entry (used like a tuple), with the resulting binned mat as the
     *         key, and the minimum coordinate of the pointcloud as the value.
     */
    private Entry<Mat, BinnedPoint> toMat(List<Translation2d> pointcloud) {
        if (pointcloud.size() <= 0) {
            System.err.println("toMat got an empty pointcloud");
            return Map.entry(new Mat(), new BinnedPoint(0, 0));
        }

        Hashtable<BinnedPoint, Integer> binnedPoints = new Hashtable<>();

        // Bin and deduplicate the points, and find the min and max coords
        BinnedPoint min = null, max = null;
        for (Translation2d point : pointcloud) {
            BinnedPoint binnedPoint = new BinnedPoint(point);

            Integer density = binnedPoints.get(binnedPoint);
            if (density == null) {
                binnedPoints.put(binnedPoint, 1);
            } else {
                binnedPoints.put(binnedPoint, density++);
            }
            // Density is currently unused

            if (min == null) {
                min = binnedPoint;
            }
            if (max == null) {
                max = binnedPoint;
            }

            min = new BinnedPoint(Math.min(binnedPoint.mBinX, min.mBinX), Math.min(binnedPoint.mBinY, min.mBinY));
            max = new BinnedPoint(Math.max(binnedPoint.mBinX, max.mBinX), Math.max(binnedPoint.mBinY, max.mBinY));
        }

        Mat binnedPointcloud = Mat.zeros(max.y() - min.y(), max.x() - min.x(), CvType.CV_8UC(1));

        // Normalize the points to have an origin at 0, 0 and put them into the mat
        for (BinnedPoint point : binnedPoints.keySet()) {
            binnedPointcloud.put(point.y() - min.y(), point.x() - min.x(), new byte[] { (byte) 0xFF });
        }
        // binnedPointcloud.put(0 - min.y(), 0 - min.x(), new byte[] { (byte) 0x80 });

        return Map.entry(binnedPointcloud, min);
    }

    private class BinnedPoint {

        /** Indicies in the bin (i.e. unitless) */
        private final int mBinX, mBinY;

        /**
         * @param point A sensor-relative point, with X and Y in meters.
         */
        public BinnedPoint(Translation2d point) {
            // Overflow possible but very unlikely
            mBinX = (int) (point.getX() / mBinWidth);
            mBinY = (int) (point.getY() / mBinWidth);
        }

        public BinnedPoint(int x, int y) {
            mBinX = x;
            mBinY = y;
        }

        @Override
        public boolean equals(Object other) {
            if (this == other) {
                return true;
            } else if (other == null) {
                return false;
            } else if (this.getClass() != other.getClass()) {
                return false;
            }

            var bp = (BinnedPoint) other;
            return bp.mBinX == this.mBinX && bp.mBinY == this.mBinY;
        }

        @Override
        public int hashCode() {
            int hashX = Integer.valueOf(mBinX).hashCode();
            int hashY = Integer.valueOf(mBinY).hashCode();
            return 31 * hashX + hashY;
        }

        public int x() {
            return mBinX;
        }

        public int y() {
            return mBinY;
        }

        /**
         * @return This binned point converted back to a Translation2d in meters, in the
         *         sensor-relative coordinate plane.
         */
        public Translation2d toTranslation2d() {
            return new Translation2d(mBinWidth * mBinX + mBinX / 2.0, mBinWidth * mBinY + mBinY / 2.0);
        }
    }
}