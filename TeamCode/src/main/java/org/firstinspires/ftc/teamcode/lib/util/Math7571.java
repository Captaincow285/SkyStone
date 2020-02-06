package org.firstinspires.ftc.teamcode.lib.util;

import org.firstinspires.ftc.teamcode.lib.movement.Point;

public class Math7571{


  public static double round(double value, int places) {
    double scale = Math.pow(10, places);
    return Math.round(value * scale) / scale;
  }

  public static double calculateHeading(double x, double y){

    return  Math.atan(y/x);
  }

  public static double calculateHeading(Point target){

    return  Math.atan(target.getX()/target.getY());
  }

  public static double calcHyp(double x, double y){

    double A = Math.atan(y/x);
    return y/Math.sin(A) * Math.sin(90);

  }

  public static double calcHyp(Point current, Point target){

    double xDelta = target.getX() - current.getX(), yDelta = target.getY() - current.getY();

    if(yDelta < 0 || xDelta < 0){
      return -Math.sqrt(Math.pow(xDelta, 2) + Math.pow(yDelta, 2));
    } else {
      return Math.sqrt(Math.pow(xDelta, 2) + Math.pow(yDelta, 2));
    }

  }

  public static double[][] multiply(double[][] a, double[][] b) {
    int m1 = a.length;
    int n1 = a[0].length;
    int m2 = b.length;
    int n2 = b[0].length;
    if (n1 != m2) throw new RuntimeException("Illegal matrix dimensions.");
    double[][] c = new double[m1][n2];
    for (int i = 0; i < m1; i++)
      for (int j = 0; j < n2; j++)
        for (int k = 0; k < n1; k++)
          c[i][j] += a[i][k] * b[k][j];
    return c;
  }

}