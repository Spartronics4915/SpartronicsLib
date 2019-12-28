package com.spartronics4915.lib.math.twodim.geometry;

import com.spartronics4915.lib.util.Interpolable;
import com.spartronics4915.lib.math.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an
 * (x, y) plane.
 */
public class Translation2d implements Interpolable<Translation2d> {

    private final double mX;
    private final double mY;

    public Translation2d() {
        mX = 0;
        mY = 0;
    }

    public Translation2d(double x, double y) {
        mX = x;
        mY = y;
    }

    public Translation2d(final Translation2d other) {
        mX = other.mX;
        mY = other.mY;
    }

    public Translation2d(final Translation2d start, final Translation2d end) {
        mX = end.mX - start.mX;
        mY = end.mY - start.mY;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm() {
        return Math.hypot(mX, mY);
    }

    public double norm2() {
        return mX * mX + mY * mY;
    }

    public double getX() {
        return mX;
    }

    public double getY() {
        return mY;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation2d translateBy(final Translation2d other) {
        return new Translation2d(mX + other.mX, mY + other.mY);
    }

    /**
     * We can also rotate Translation2d's. See:
     * https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(mX * rotation.getCos() - mY * rotation.getSin(), mX * rotation.getSin() + mY * rotation.getCos());
    }

    public Rotation2d direction() {
        return new Rotation2d(mX, mY, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation2d inverse() {
        return new Translation2d(-mX, -mY);
    }

    @Override
    public Translation2d interpolate(final Translation2d endValue, double t) {
        if (t <= 0) {
            return new Translation2d(this);
        } else if (t >= 1) {
            return new Translation2d(endValue);
        }
        return extrapolate(endValue, t);
    }

    public Translation2d extrapolate(final Translation2d other, double x) {
        return new Translation2d(x * (other.mX - mX) + mX, x * (other.mY - mY) + mY);
    }

    public Translation2d scale(double s) {
        return new Translation2d(mX * s, mY * s);
    }

    public boolean epsilonEquals(final Translation2d other, double epsilon) {
        return Util.epsilonEquals(getX(), other.getX(), epsilon) && Util.epsilonEquals(getY(), other.getY(), epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(mX) + "," + fmt.format(mY) + ")";
    }

    /**
     * @return this · b
     */
    public double dot(final Translation2d b) {
        return this.mX * b.mX + this.mY * b.mY;
    }

    /**
     * @return this ⨯ b (also the determinant of a 2x2 matrix where
     *         <code>this</code> is in the first column and b is in the second
     *         column)
     */
    public double cross(final Translation2d b) {
        return this.mX * b.mY - this.mY * b.mX;
    }

    public Rotation2d getAngle(final Translation2d b) {
        // The below works because a · b = |a||b|cosθ and a ⨯ b = |a||b|sinθ. Because
        // both sin and cos are multiplied by the same number -- |a||b| are
        // the same (and always nonnegative) -- the ratio is the same.
        return Rotation2d.fromRadians(Math.atan2(this.cross(b), this.dot(b)));
    }

    // This class could implement TrajectoryState because it has a distance
    // method, but you would never use a Translation2d as a TrajectoryState.
    // The above applies to Rotation2d and Twist2d as well.
    public double getDistance(final Translation2d other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Translation2d))
            return false;
        return getDistance((Translation2d) other) < Util.kEpsilon;
    }

    public Translation2d getTranslation() {
        return this;
    }
}
