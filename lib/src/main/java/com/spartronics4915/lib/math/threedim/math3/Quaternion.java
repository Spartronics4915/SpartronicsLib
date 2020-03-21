package com.spartronics4915.lib.math.threedim.math3;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;

/**
 *  Robust+compact representation of 3D rotation
 *
 *  Quaternions w+ix+jy+kz are represented as [w, x, y, z].
 *  see: 
 *      ttps://eater.net/quaternions/intro
 *      https://www.youtube.com/watch?v=jlskQDR8-bY  (Mathoma)
 *      https://www.youtube.com/watch?v=d4EgbgTm0Bg (3Blue1Brown)
 *      https://eater.net/quaternions (Ben Eater)
 */

public class Quaternion
{
    static final double kEpsilon = 1e-9;

    /* factory static methods -----------------------------------------------*/

    /* methods -----------------------------------------------*/
    private Rotation mQ; // internally Rotation stores a quaternion

    /**
     * 4 double contructor - each of the 4 quaternion components
     * @param q0
     * @param q1
     * @param q2
     * @param q3
     */
    public Quaternion(double q0, double q1, double q2, double q3)
    {
        this.mQ = new Rotation(q0, q1, q2, q3, false/*normalize*/);  
    }

    /**
     * default contructor, produces identity
     */
    public Quaternion()
    {
        this(1, 0, 0, 0);
    }

    /**
     * angle/axis contructor
     * @param angle - measured in degrees
     * @param axis
     */
    public Quaternion(double angle, Vec3 axis)
    {
        this.mQ = new Rotation(axis.asVector3D(), Math.toRadians(angle),
                            RotationConvention.FRAME_TRANSFORM);
    }

    /**
     * euler-angle constructor
     * @param ai
     * @param aj
     * @param ak
     * @param axes
     */
    public Quaternion(double ai, double aj, double ak, String axes)
    {
        assert (false); // XXX: implement me if need-be
    }

    /**
     * 
     * @param a - an Affine3 to decompose
     * @param precise - if true, input is assumed to be a precise rotation 
     *   matrix and a faster algorithm is used. If in doubt, set to false.
     */
    public Quaternion(Matrix3 a)
    {
        this(a, false);
    }

    public Quaternion(Matrix3 a, boolean precise)
    {
        Matrix3 at = a.transpose(); // alternate rotation rep for opencv compat
        this.mQ = new Rotation(at.getData(), precise ? 1e-9 : 1e-3);
    }

    public Quaternion(Matrix4 a, boolean precise)
    {
        Matrix3 a3 = new Matrix3(a).transpose();// alternate rotation rep for opencv compat
        this.mQ = new Rotation(a3.getData(), precise ? 1e-9 : 1e-3);
    }

    /**
     * converts the current quaternion values into 3x3 matrix form.
     * nb: translation and projection fields are 0.
     * nb2: we transpose to obtain the opencv concatenation sense.
     * @return
     */
    public Matrix3 asMatrix3()
    {
        Matrix3 m = new Matrix3(mQ.getMatrix());
        return m.transpose();
    }

    public Matrix4 asMatrix4()
    {
        return new Matrix4(this.asMatrix3());
    }

    public boolean equals(Quaternion rhs)
    {
        return this.equals(rhs, 1e-9);
    }

    public boolean equals(Quaternion rhs, double epsilon)
    {
        if (Math.abs(this.mQ.getQ0() - rhs.mQ.getQ0()) > epsilon)
            return false;
        if (Math.abs(this.mQ.getQ1() - rhs.mQ.getQ1()) > epsilon)
            return false;
        if (Math.abs(this.mQ.getQ2() - rhs.mQ.getQ2()) > epsilon)
            return false;
        if (Math.abs(this.mQ.getQ3() - rhs.mQ.getQ3()) > epsilon)
            return false;
        return true;
    }

    @Override
    public String toString()
    {
        return String.format("Q %g %g %g %g", 
            this.mQ.getQ0(), this.mQ.getQ1(), this.mQ.getQ2(), this.mQ.getQ3());
    }

    public void print()
    {
        System.out.println(this.toString());
    }
}
