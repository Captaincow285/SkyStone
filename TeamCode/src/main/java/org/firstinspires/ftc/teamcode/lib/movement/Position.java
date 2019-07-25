package org.firstinspires.ftc.teamcode.lib.movement;

public class Position{

    private Point current, target;

    double x, y, a;

    public Position(double x, double y, double a){

        this.x = x;
        this.y = y;
        this.a = a;

    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getA(){
        return a;
    }

    public Point getCurrent(){
        return current;
    }

    public Point getTarget(){
        return target;
    }

    public void setCurrent(Point point){

        current = point;

    }

    public String toString(){

        return ("(" + current.getX() + ", " + current.getY() + ")");
    }


}