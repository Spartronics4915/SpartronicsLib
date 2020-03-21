package com.spartronics4915.lib.math.threedim.math3;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/* Vec3 is a thin vaneer atop Vector3D.  Its sole purpose
 * is to make clients of this library not have to worry about
 * importing. We conflate the geometric interpretation of
 * points and vectors in this class and reduce the exposure.
 * We also support mutability.
 */

public class Vec3
{
    // quell seriazable warning
    private static final long serialVersionUID = 8264857513233341906L;

    public final static Vec3 XAxis = new Vec3(1, 0, 0);
    public final static Vec3 YAxis = new Vec3(0,1,0);
    public final static Vec3 ZAxis = new Vec3(0,0,1);
    public final static Vec3 ZeroPt = new Vec3(0,0,0);

    private Vector3D mVec3;

    public Vec3()
    {
        mVec3 = new Vector3D(0,0,0);
    }

    protected Vec3(Vector3D v)
    {
        mVec3 = v;
    }

    protected Vector3D asVector3D()
    {
        return mVec3;
    }

    public Vec3(double x, double y, double z)
    {
        mVec3 = new Vector3D(x, y, z);
    }

    public Vec3(Vec3 v)
    {
        mVec3 = new Vector3D(v.getX(), v.getY(), v.getZ());
    }

    public Vec3(double[] v)
    {
        this(v[0], v[1], v[2]);
        assert v.length == 3;
    }

    public double getX()
    {
        return mVec3.getX();
    }

    public double getY()
    {
        return mVec3.getY();
    }

    public double getZ()
    {
        return mVec3.getZ();
    }

    public Vec3 add(final Vec3 rhs)
    {
        return new Vec3(this.getX()+rhs.getX(), 
                       this.getY()+rhs.getY(), 
                       this.getZ()+rhs.getZ());
    }

    public Vec3 add(double x, double y, double z)
    {
        return new Vec3(this.getX()+x, this.getY()+y, this.getZ()+z);
    }

    public Vec3 add(double []v)
    {
        assert v.length == 3;
        return this.add(v[0], v[1], v[2]);
    }

    public Vec3 subtract(final Vec3 rhs)
    {
        return new Vec3(this.getX()-rhs.getX(), 
                        this.getY()-rhs.getY(), 
                        this.getZ()-rhs.getZ());
    }

    public Vec3 subtract(double x, double y, double z)
    {
        return new Vec3(this.getX()-x, this.getY()-y, this.getZ()-z);
    }

    public Vec3 subtract(double [] v)
    {
        assert v.length == 3;
        return this.subtract(v[0], v[1], v[2]);
    }

    // mutable
    public void multiply(double f)
    {
        this.mVec3 = this.mVec3.scalarMultiply(f);
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
        Vec3 nv = new Vec3(this.getX(), this.getY(), 0).asUnit();
        return Math.toDegrees(Math.atan2(nv.getY(), nv.getX()));
    }

    public double angleOnXZPlane()
    {
        Vec3 nv = new Vec3(this.getX(), 0, this.getZ()).asUnit();
        return Math.toDegrees(Math.atan2(nv.getZ(), nv.getX()));
    }

    public double dot(final Vec3 rhs)
    {
        return Vector3D.dotProduct(this.mVec3, rhs.mVec3);
    }

    public Vec3 cross(final Vec3 rhs)
    {
        Vector3D x = Vector3D.crossProduct(this.mVec3, rhs.mVec3);
        return new Vec3(x);
    }

    public boolean equals(final Vec3 rhs)
    {
        return this.equals(rhs, 1e-9);
    }

    public boolean equals(final Vec3 rhs, double epsilon)
    {
        if(Math.abs(this.getX() - rhs.getX()) > epsilon)
            return false;
        if(Math.abs(this.getY() - rhs.getY()) > epsilon)
            return false;
        if(Math.abs(this.getZ() - rhs.getZ()) > epsilon)
            return false;
        return true;
    }

    /**
     * @return length of 3D vector. aka: distance of 3d point from origin.
     */
    public double length()
    {
        return this.mVec3.getNorm(); // L2 norm, L1 is manhattan
    }

    /**
     * @return length of 2D vector on XY plane
     */
    public double lengthXY()
    {
        return Math.hypot(this.mVec3.getX(), this.mVec3.getY());
    }

    // mutable
    public void normalize()
    {
        this.mVec3 = this.mVec3.normalize();
    }

    public Vec3 asUnit()
    {
        return new Vec3(this.mVec3.normalize());
    }

    public Vec3 asOpposite()
    {
        return new Vec3(this.mVec3.negate());
    }

    public String asPointString()
    {
        return String.format("pt3 %g %g %g", 
                    this.getX(), this.getY(), this.getZ());
    }

    public String asDirectionString()
    {
        return String.format("vec3 %g %g %g",
                    this.getX(), this.getY(), this.getZ());
    }

    public Matrix3 outerProduct(Vec3 rhs)
    {
        Matrix3 result = new Matrix3();
        double data[][] = result.getDataRef();
        data[0][0] = this.getX() * rhs.getX();
        data[0][1] = this.getY() * rhs.getX();
        data[0][2] = this.getZ() * rhs.getX();
        data[1][0] = this.getX() * rhs.getY();
        data[1][1] = this.getY() * rhs.getY();
        data[1][2] = this.getZ() * rhs.getY();
        data[2][0] = this.getX() * rhs.getZ();
        data[2][1] = this.getY() * rhs.getZ();
        data[2][2] = this.getZ() * rhs.getZ();
        return result;
    }
    
}