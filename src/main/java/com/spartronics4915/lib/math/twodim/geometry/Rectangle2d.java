package com.spartronics4915.lib.math.twodim.geometry;

import java.util.Arrays;

import com.spartronics4915.lib.math.Util;

public class Rectangle2d {

    private double mX, mY;
    private double mWidth, mHeight;

    /**
     * @param x X coordinate of the bottom left hand corner.
     * @param y Y coordinate of the bottom left hand corner.
     */
    public Rectangle2d(double x, double y, double width, double height) {
        mX = x;
        mY = y;
        mWidth = width;
        mHeight = height;
    }

    public Rectangle2d(Translation2d one, Translation2d two) {
        double minX, minY, maxX, maxY;
        minX = Math.min(one.getX(), two.getX());
        minY = Math.min(one.getY(), two.getY());
        maxX = Math.max(one.getX(), two.getX());
        maxY = Math.max(one.getY(), two.getY());

        mX = minX;
        mY = minY;
        mWidth = maxX - minX;
        mHeight = maxY - minY;
    }

    public Rectangle2d(Translation2d... points) {
        double minX, minY, maxX, maxY;
        minX = Arrays.stream(points).map((t) -> t.getX()).min(Double::compare).orElseThrow();
        minY = Arrays.stream(points).map((t) -> t.getY()).min(Double::compare).orElseThrow();
        maxX = Arrays.stream(points).map((t) -> t.getX()).max(Double::compare).orElseThrow();
        maxY = Arrays.stream(points).map((t) -> t.getY()).max(Double::compare).orElseThrow();

        mX = minX;
        mY = minY;
        mWidth = maxX - minX;
        mHeight = maxY - minY;
    }

    public double x() {
        return mX;
    }

    public double y() {
        return mY;
    }

    public double width() {
        return mWidth;
    }

    public double height() {
        return mHeight;
    }

    public Translation2d getTopLeft() {
        return new Translation2d(mX, mY + mHeight);
    }

    public Translation2d getTopRight() {
        return new Translation2d(mX + mWidth, mY + mHeight);
    }

    public Translation2d getBottomLeft() {
        return new Translation2d(mX, mY);
    }

    public Translation2d getBottomRight() {
        return new Translation2d(mX + mWidth, mY);
    }

    public Translation2d getCenter() {
        return new Translation2d(mX + mWidth / 2, mX + mWidth / 2);
    }

    public boolean isIn(Rectangle2d r) {
        return mX < r.mX + r.mWidth && mX + mWidth > r.mX && mY < r.mY + r.mHeight && mY + mHeight > r.mY;
    }

    public boolean contains(Translation2d t) {
        return (t.getX() > mX && t.getX() < mX + mWidth) && (t.getY() > mY && t.getY() < mY + mHeight);
    }

    public boolean doesCollide(Rectangle2d rectangle, Translation2d translation) {
        if (Util.epsilonEquals(0, translation.getX()) && Util.epsilonEquals(0, translation.getY()))
            return false;
        // Check if it's even in range
        Rectangle2d boxRect = new Rectangle2d(
            rectangle.getTopLeft(), rectangle.getBottomRight(),
            rectangle.getTopLeft().translateBy(translation), rectangle.getBottomRight().translateBy(translation)
        );
        if (!boxRect.isIn(this)) return false;
        
        // AABB collision
        // Calculate distances
        double xInvEntry, xInvExit, yInvEntry, yInvExit;
        if (translation.getX() > 0.0) {
            xInvEntry = (mX - (rectangle.mX + rectangle.mWidth));
            xInvExit = ((mX + mWidth) - rectangle.mX);
        } else {
            xInvEntry = ((mX + mWidth) - rectangle.mX);
            xInvExit = (mX - (rectangle.mX + rectangle.mWidth));
        }
        if (translation.getY() > 0.0) {
            yInvEntry = (mY - (rectangle.mY + rectangle.mHeight));
            yInvExit = ((mY + mHeight) - rectangle.mY);
        } else {
            yInvEntry = ((mY + mHeight) - rectangle.mY);
            yInvExit = (mY - (rectangle.mY + rectangle.mHeight));
        }
        // Find time of collisions
        double xEntry, xExit, yEntry, yExit;
        if (Util.epsilonEquals(0, translation.getX())) {
            xEntry = Double.NEGATIVE_INFINITY;
            xExit = Double.POSITIVE_INFINITY;
        } else {
            xEntry = xInvEntry / translation.getX();
            xExit = xInvExit / translation.getX();
        }
        if (Util.epsilonEquals(0, translation.getY())) {
            yEntry = Double.NEGATIVE_INFINITY;
            yExit = Double.POSITIVE_INFINITY;
        } else {
            yEntry = yInvEntry / translation.getY();
            yExit = yInvExit / translation.getY();
        }
        double entryTime = Math.max(xEntry, yEntry);
        double exitTime = Math.min(xExit, yExit);

        return entryTime <= exitTime && (xEntry >= 0.0 || yEntry >= 0.0) && (xEntry < 1.0 || yEntry < 1.0);
    }
}