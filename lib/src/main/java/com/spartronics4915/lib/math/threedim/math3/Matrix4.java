package com.spartronics4915.lib.math.threedim.math3;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;

public class Matrix4 extends Array2DRowRealMatrix
{
    // quell serializable warning (currently unimportant)
    private static final long serialVersionUID = 1L;
    private static final String kFmt = "Matrix4\n"+
                            "  %8.4f, %8.4f, %8.4f, %8.4f\n"+
                            "  %8.4f, %8.4f, %8.4f, %8.4f\n"+
                            "  %8.4f, %8.4f, %8.4f, %8.4f\n"+
                            "  %8.4f, %8.4f, %8.4f, %8.4f";

    public static Matrix4 identity()
    {
        Matrix4 x = new Matrix4();
        double data[][] = x.getDataRef();
        data[0][0] = 1;
        data[1][1] = 1;
        data[2][2] = 1;
        data[3][3] = 1;
        return x;
    }

    public Matrix4()
    {
        super(4,4);
    }

    public Matrix4(double[][] data)
    {
        super(data);
    }

    /**
     * Construct a Matrix4 from a 3D Matrix3 rotation matrix.
     * @param m
     */
    public Matrix4(Matrix3 m)
    {
        super(4,4);
        double data[][] = this.getDataRef();
        m.copySubMatrix(0, 2, 0, 2, data);
        data[3][3] = 1.;
    }

    public boolean equals(final Matrix4 rhs, double epsilon)
    {
        double a[][] = this.getDataRef();
        double b[][] = rhs.getDataRef();
        if(Math.abs(a[0][0] - b[0][0]) > epsilon) return false;
        if(Math.abs(a[0][1] - b[0][1]) > epsilon) return false;
        if(Math.abs(a[0][2] - b[0][2]) > epsilon) return false;
        if(Math.abs(a[0][3] - b[0][3]) > epsilon) return false;
        if(Math.abs(a[1][0] - b[1][0]) > epsilon) return false;
        if(Math.abs(a[1][1] - b[1][1]) > epsilon) return false;
        if(Math.abs(a[1][2] - b[1][2]) > epsilon) return false;
        if(Math.abs(a[1][3] - b[1][3]) > epsilon) return false;
        if(Math.abs(a[2][0] - b[2][0]) > epsilon) return false;
        if(Math.abs(a[2][1] - b[2][1]) > epsilon) return false;
        if(Math.abs(a[2][2] - b[2][2]) > epsilon) return false;
        if(Math.abs(a[2][3] - b[2][3]) > epsilon) return false;
        if(Math.abs(a[3][0] - b[3][0]) > epsilon) return false;
        if(Math.abs(a[3][1] - b[3][1]) > epsilon) return false;
        if(Math.abs(a[3][2] - b[3][2]) > epsilon) return false;
        if(Math.abs(a[3][3] - b[3][3]) > epsilon) return false;
        return true;
    }

    public Matrix4 invert()
    {
        return new Matrix4(MatrixUtils.inverse(this).getData());
    }

    public Matrix4 transpose()
    {
        return new Matrix4(super.transpose().getData());
    }
    
    public String toString()
    {
        double data[][] = this.getDataRef();
        return String.format(kFmt, 
                            data[0][0], data[0][1], data[0][2], data[0][3],
                            data[1][0], data[1][1], data[1][2], data[1][3],
                            data[2][0], data[2][1], data[2][2], data[2][3],
                            data[3][0], data[3][1], data[3][2], data[3][3]
                            );
    }

    public void print()
    {
        System.out.println(this.toString());
    }
}