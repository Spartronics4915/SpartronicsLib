package com.spartronics4915.lib.math.threedim.ejml;

import org.ejml.data.DMatrix3; // Vec3
import org.ejml.data.DMatrix3x3; // Vec3
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.ejml.dense.fixed.NormOps_DDF3;


/* Vec3 is a thin vaneer atop DMatrix3.  Its sole purpose
 * is to make clients of this library not have to worry about
 * importing DMatrix3. We also offer a small set of conveniece
 * methods.
 */

public class Vec3 extends DMatrix3
{
    private static final long serialVersionUID = -6151934040052916311L;

    public final static Vec3 XAxis = new Vec3(1,0,0);
    public final static Vec3 YAxis = new Vec3(0,1,0);
    public final static Vec3 ZAxis = new Vec3(0,0,1);
    public final static Vec3 ZeroPt = new Vec3(0,0,0);

    public Vec3()
    {
        super();
    }

    public Vec3(double x, double y, double z)
    {
        super(x, y, z);
    }

    public Vec3(Vec3 v)
    {
        super(v);
    }

    public Vec3(double[] v)
    {
        super(v[0], v[1], v[2]);
        assert v.length == 3;
    }

    public double getX()
    {
        return this.a1;
    }

    public double getY()
    {
        return this.a2;
    }

    public double getZ()
    {
        return this.a3;
    }

    public Vec3 add(final Vec3 rhs)
    {
        return new Vec3(this.a1+rhs.a1, this.a2+rhs.a2, this.a3+rhs.a3);
    }

    public Vec3 add(double x,  double y, double z)
    {
        return new Vec3(this.a1+x, this.a2+y, this.a3+z);
    }

    public Vec3 add(double []v)
    {
        assert v.length == 3;
        return this.add(v[0], v[1], v[2]);
    }

    public Vec3 subtract(final Vec3 rhs)
    {
        return new Vec3(this.a1-rhs.a1, this.a2-rhs.a2, this.a3-rhs.a3);
    }

    public Vec3 subtract(double x, double y, double z)
    {
        return new Vec3(this.a1-x, this.a2-y, this.a3-z);
    }

    public Vec3 subtract(double [] v)
    {
        assert v.length == 3;
        return this.subtract(v[0], v[1], v[2]);
    }

    public void multiply(double f)
    {
        this.a1 *= f;
        this.a2 *= f;
        this.a3 *= f;
    }

    /**
     * compute the angle between two 3d vectors
     * @param Vec3
     * @return angle in degrees. Sign of angle is always positive.
     */
    public double angleWith(final Vec3 rhs)
    {
        double d = this.asUnit().dot(rhs.asUnit());
        double rads = Math.acos(d > 1.0 ? 1 : d < -1. ? -1. : d);
        return Math.toDegrees(rads);
    }

    public double angleOnXYPlane()
    {
        Vec3 nv = new Vec3(this.a1, this.a2, 0).asUnit();
        return Math.toDegrees(Math.atan2(nv.a2, nv.a1));
    }

    public double dot(final Vec3 rhs)
    {
        return this.a1*rhs.a1 + this.a2*rhs.a2 + this.a3*rhs.a3;
    }

    public Vec3 cross(final Vec3 rhs)
    {
        Vec3 result = new Vec3();
        result.a1 = this.a2 * rhs.a3 - rhs.a2 * this.a3;
        result.a2 = rhs.a1 * this.a3 - this.a1 * rhs.a3;
        result.a3 = this.a1 * rhs.a2 - rhs.a1 * this.a2;
        return result;
    }

    public boolean equals(final Vec3 rhs)
    {
        return this.equals(rhs, 1e-9);
    }

    public boolean equals(final Vec3 rhs, double epsilon)
    {
        if(Math.abs(this.a1 - rhs.a1) > epsilon)
            return false;
        if(Math.abs(this.a2 - rhs.a2) > epsilon)
            return false;
        if(Math.abs(this.a3 - rhs.a3) > epsilon)
            return false;
        return true;
    }

    /**
     * @return lenght of 3D vector. aka: distance of 3d point from origin.
     */
    public double length()
    {
        return NormOps_DDF3.normF(this);
    }

    /**
     * @return - length of xy components of vector.
     *   aka: distance of xy point from origin.
     */
    public double lengthXY()
    {
        return Math.sqrt(this.a1*this.a1 + this.a2*this.a2);
    }

    public void normalize()
    {
        NormOps_DDF3.normalizeF(this);
    }

    public Vec3 asUnit()
    {
        Vec3 result = new Vec3(this);

        result.normalize();
        return result;
    }

    public Vec3 asOpposite()
    {
        return new Vec3(-this.a1, -this.a2, -this.a3);
    }

    public String asPointString()
    {
        return String.format("pt3 %g %g %g", this.a1, this.a2, this.a3);
    }

    public String asDirectionString()
    {
        return String.format("vec3 %g %g %g", this.a1, this.a2, this.a3);
    }

    public DMatrix3x3 outerProduct(Vec3 rhs)
    {
        DMatrix3x3 result = new DMatrix3x3();
        result.a11 = this.a1 * rhs.a1;
        result.a12 = this.a2 * rhs.a1;
        result.a13 = this.a3 * rhs.a1;
        result.a21 = this.a1 * rhs.a2;
        result.a22 = this.a2 * rhs.a2;
        result.a23 = this.a3 * rhs.a2;
        result.a31 = this.a1 * rhs.a3;
        result.a32 = this.a2 * rhs.a3;
        result.a33 = this.a3 * rhs.a3;
        return result;
    }
    
}