package com.spartronics4915.lib.math.threedim;

import com.spartronics4915.lib.math.threedim.ejml.*;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.ejml.data.Complex_F64;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.fixed.CommonOps_DDF4;
import org.ejml.dense.row.decomposition.eig.SymmetricQRAlgorithmDecomposition_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.junit.jupiter.api.Test;

class EjmlTests
{
    static final double kEpsilon = 1e-9;
    @Test
    void testEjmlDMatrix()
    {
        DMatrix4x4 m1 = new DMatrix4x4();
        CommonOps_DDF4.setIdentity(m1);
        m1.a33 *= 3;
        assertEquals(m1.a33,  m1.get(2,2));  //  the get method is index-origin 0
        m1.set(0, 0, 10); //  the set method is index-origin 0
        assertEquals(m1.a11, 10, kEpsilon);  

        // is data shared or copied? -> copied!
        DMatrixRMaj m2 = new DMatrixRMaj(m1);
        m2.set(1,1, 33);
        // m2.print();
        assertEquals(m2.get(1,1), 33, kEpsilon);
        assertEquals(m1.get(1,1), 1, kEpsilon); 

    }

    void matToQuaternion(DMatrix4x4 m, double q[], double w[])
    {
        // m.print();
        DMatrix4x4 k = new DMatrix4x4();
        k.a11 = m.a11 - m.a22 - m.a33; 
        k.a12 = 0; 
        k.a13 = 0; 
        k.a14 = 0;
        k.a21 = m.a12 + m.a21; 
        k.a22 = m.a22 - m.a11 - m.a33; 
        k.a23 = 0; 
        k.a24 = 0;
        k.a31 = m.a13 + m.a31;
        k.a32 = m.a23 + m.a32;
        k.a33 = m.a33 - m.a11 - m.a22;
        k.a34 = 0;
        k.a41 = m.a32 - m.a23; 
        k.a42 = m.a13 - m.a31; 
        k.a43 = m.a21 - m.a12; 
        k.a44 = m.a11 + m.a22 + m.a33; 
        CommonOps_DDF4.scale(1./3, k);
        // This shorthand represents 0 as placeholders for the symmetric
        // components.  numpy.linalg.eigh defaults to using the lower
        // diagonal, while it appears the ejml uses the entire matrix;
        k.a12 = k.a21;
        k.a13 = k.a31;
        k.a14 = k.a41;
        k.a23 = k.a32;
        k.a24 = k.a42;
        k.a34 = k.a43;
        // k.print();
        DMatrixRMaj rm2 = new DMatrixRMaj(k);
        EigenDecomposition<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(true, true);
        // EigenDecomposition<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(4, true);
        eig.decompose(rm2);
        // find eigenvector associated with largest eigenvalue
        if(eig instanceof SymmetricQRAlgorithmDecomposition_DDRM)
        {
            DMatrixRMaj[] VT = {null, null, null, null};
            int argMax = -1; 
            double magMax2 = 0;
            for(int i=0;i<eig.getNumberOfEigenvalues(); i++)
            {
                Complex_F64 v = ((SymmetricQRAlgorithmDecomposition_DDRM)eig).getEigenvalue(i);
                assert(v.isReal());
                w[i] = v.getReal();
                VT[i] = eig.getEigenVector(i);
                double mag2 = v.getMagnitude2();
                if(mag2 > magMax2)
                {
                    magMax2 = mag2;
                    argMax = i;
                }
            }
            assert(argMax > -1);

            // re-arrange
            q[0] = VT[argMax].get(3, 0);
            q[1] = VT[argMax].get(0, 0);
            q[2] = VT[argMax].get(1, 0);
            q[3] = VT[argMax].get(2, 0);

            if(q[0] < 0)
            {
                q[0] *= -1;
                q[1] *= -1;
                q[2] *= -1;
                q[3] *= -1;
            }
        }
        else
            assert(false);
    }

    @Test
    void testEig()
    {
        double q[] = {0,0,0,0};
        double w[] = {0,0,0,0};
        DMatrix4x4 m = Affine3.fromRotation(33, new Vec3(1,1,1)).asMatrix();
        // m:
        // [ 0.89244705, -0.26067102,  0.36822397,  0.        ],
        // [ 0.36822397,  0.89244705, -0.26067102,  0.        ],
        // [-0.26067102,  0.36822397,  0.89244705,  0.        ],
        // [ 0.        ,  0.        ,  0.        ,  1.        ]
        // k:
        // [-0.29748235,  0.        ,  0.        ,  0.        ],
        // [ 0.03585098, -0.29748235,  0.        ,  0.        ],
        // [ 0.03585098,  0.03585098, -0.29748235,  0.        ],
        // [ 0.20963166,  0.20963166,  0.20963166,  0.89244705]
        this.matToQuaternion(m, q, w);
        // System.out.println(String.format("Here's w: %g %g %g %g", w[0], w[1], w[2], w[3]));
        // System.out.println(String.format("Here's q: %g %g %g %g", q[0], q[1], q[2], q[3]));
        assertEquals(q[0], .958820, 1e-5);
        assertEquals(q[1], .163976, 1e-5);
        assertEquals(q[2], .163976, 1e-5);
        assertEquals(q[3], .163976, 1e-5);

        m = Affine3.fromRotation(45, new Vec3(1,0,0)).asMatrix();
        this.matToQuaternion(m, q, w);
        assertEquals(q[0], .923880, 1e-5);
        assertEquals(q[1], .382683, 1e-5);
        assertEquals(q[2], .0, 1e-5);
        assertEquals(q[3], .0, 1e-5);
        // System.out.println(String.format("Here's w: %g %g %g %g", w[0], w[1], w[2], w[3]));
        // System.out.println(String.format("Here's q: %g %g %g %g", q[0], q[1], q[2], q[3]));
    }
}