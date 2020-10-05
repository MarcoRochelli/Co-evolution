package it.units.erallab;

import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealMatrixFormat;

import java.text.DecimalFormat;
import java.util.function.Supplier;
import java.util.logging.Logger;

public class MakeEigenDecompositionExplode {

  private static final Logger L = Logger.getLogger(MakeEigenDecompositionExplode.class.getName());

  public static void main(String[] args) {
    int rows = 3;
    int columns = 3;

    double[][] matrix1 = {  // can not not be diagonalized
        {1, 1, 2},
        {0, -1, -1},
        {0, 1, 1}
    };
    RealMatrix bomb1 = MatrixUtils.createRealMatrix(matrix1);
    double[][] matrix2 = {  // zeros
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    RealMatrix bomb2 = MatrixUtils.createRealMatrix(matrix2);

    double[][] matrix3 = {  // hessenberg matrix ???
        {0, 1, 0},
        {0, 0, 1},
        {1, 0, 0}
    };
    RealMatrix bomb3 = MatrixUtils.createRealMatrix(matrix3);

    double[][] matrix4 = {  // zero determinant
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    RealMatrix bomb4 = MatrixUtils.createRealMatrix(matrix4);

    for (int x = 0; x < rows; x++) {  // to print input matrix
      for (int y = 0; y < columns; y++) {
        System.out.print(matrix1[x][y] + " ");
      }
      System.out.println();
    }


    System.out.println(bomb1);
    System.out.println();
    /* should i format numbers?? bo i do not know how they are long or short ecc.
    DecimalFormat df = new DecimalFormat("#0.00");
    RealMatrixFormat TABLE_FORMAT = new RealMatrixFormat("", "", "", "\n", "", ", ", df );
     */

    RealMatrixFormat TABLE_FORMAT = new RealMatrixFormat("", "", "", "\n", "", ", ");
    System.out.println(TABLE_FORMAT.format(bomb1));

    EigenDecomposition eig1 = new EigenDecomposition(bomb1);
    RealMatrix B = eig1.getV();
    RealMatrix D = eig1.getD();


    EigenDecomposition eig2 = new EigenDecomposition(bomb2);
    EigenDecomposition eig3 = new EigenDecomposition(bomb3);
    EigenDecomposition eig4 = new EigenDecomposition(bomb4);
    System.out.println(eig1.getD());
    System.out.println(eig1.getV());
    System.out.println(eig1.getV());

  }
}
