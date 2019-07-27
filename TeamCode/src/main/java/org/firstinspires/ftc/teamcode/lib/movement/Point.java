package org.firstinspires.ftc.teamcode.lib.movement;

/**
 * represents a Point(x,y)
 */
public class Point{

  public double x, y;

  public Point(double x, double y){

    this.x = x;
    this.y = y;

  }

  public void setX(double x){
    this.x = x;
  }

  public void setY(double y){
    this.y = y;
  }

  public double getX(){
    return x;
  }

  public double getY(){
    return y;
  }

  public String toString(){

    return ("(" + x + ", " + y + ")");
  }


}