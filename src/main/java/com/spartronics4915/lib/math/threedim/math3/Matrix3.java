package com.spartronics4915.lib.math.threedim.math3;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;

public class Matrix3 extends Array2DRowRealMatrix
{
    // quell serializable warning (currently unimportant)
    private static final long serialVersionUID = 1L;
    private static final String kFmt = "Matrix3\n"+
                            "  %8.4f, %8.4f, %8.4f\n"+
                            "  %8.4f, %8.4f, %8.4f\n"+
                            "  %8.4f, %8.4f, %8.4f";

    public static Matrix3 identity()
    {
        Matrix3 x = new Matrix3();
        double data[][] = x.getDataRef();
        data[0][0] = 1;
        data[1][1] = 1;
        data[2][2] = 1;
        return x;
    }

    public Matrix3(double [][]data)
    {
        super(data);
    }

    public Matrix3()
    {
        super(3, 3);
    }

    public Matrix3(Matrix4 m4)
    {
        super(3,3);
        double data[][] = this.getDataRef();
        m4.copySubMatrix(0, 2, 0, 2, data);
    }
    
    public Matrix3 transpose()
    {
        return new Matrix3(super.transpose().getData());
    }

    public boolean equals(Matrix3 rhs, double epsilon)
    {
        double a[][] = this.getDataRef();
        double b[][] = rhs.getDataRef();
        if(Math.abs(a[0][0] - b[0][0]) > epsilon) return false;
        if(Math.abs(a[0][1] - b[0][1]) > epsilon) return false;
        if(Math.abs(a[0][2] - b[0][2]) > epsilon) return false;
        if(Math.abs(a[1][0] - b[1][0]) > epsilon) return false;
        if(Math.abs(a[1][1] - b[1][1]) > epsilon) return false;
        if(Math.abs(a[1][2] - b[1][2]) > epsilon) return false;
        if(Math.abs(a[2][0] - b[2][0]) > epsilon) return false;
        if(Math.abs(a[2][1] - b[2][1]) > epsilon) return false;
        if(Math.abs(a[2][2] - b[2][2]) > epsilon) return false;
        return true;
    }

    public String toString()
    {
        double data[][] = this.getDataRef();
        return String.format(kFmt, 
                            data[0][0], data[0][1], data[0][2],
                            data[1][0], data[1][1], data[1][2],
                            data[2][0], data[2][1], data[2][2]);
    }

    public void print()
    {

        System.out.println(this.toString());
    }
}