package com.spartronics4915.lib.math.twodim.trajectory;

import java.util.Objects;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class Line extends org.apache.commons.math3.geometry.euclidean.twod.Line {

    private final Vector2D mP1, mP2;

    public Line(Vector2D p1, Vector2D p2, double tolerance) {
        super(p1, p2, tolerance);
        mP1 = p1;
        mP2 = p2;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null)
            return false;
        if (!(o instanceof Line))
            return false;
        // If the vectors are the same then the lines are the same too
        // The Apache commons line class doesn't have an equals method
        return this.getAngle() == ((Line) o).getAngle() && this.getOriginOffset() == ((Line) o).getOriginOffset();
    }

    @Override
    public int hashCode() {
        return (int) (Math.random() * 1000000.0);
    }

}