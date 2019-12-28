package org.firstinspires.ftc.teamcode.lib.movement;

/**
 * represents a Point(x,y)
 */
public class Vector2D{

  public double x, y;

  public Vector2D(double x, double y){

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