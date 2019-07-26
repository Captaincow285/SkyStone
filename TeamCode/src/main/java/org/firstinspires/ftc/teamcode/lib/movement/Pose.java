package org.firstinspires.ftc.teamcode.lib.movement;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.strafeConstant;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldYPosition;

public class Pose{

    public static double x, y, a;

    private static double circumfrenceOfWheel = 15.71;
    private static double ticksPerRev = 1440;
    private static double cmPerTick = circumfrenceOfWheel/ticksPerRev;

    //public static double wheelLeftLast = 0.0;
    public static double wheelRightLast = 0.0;
    public static double wheelAuxLast = 0.0;

    public static double lastAngle = 0;

    public Pose(double x, double y, double a){

        this.x = x;
        this.y = y;
        this.a = a;

    }

    public static void setPose(Pose newPose){
        x = newPose.x;
        y = newPose.y;
        a = newPose.a;
    }

    public static void setPose(double newX, double newY, double newA){
        x = newX;
        y = newY;
        a = newA;
    }

    //dX = e0 - (dTheta * y0), dY = e1 - (dTheta * x0), and dTheta = imu_delta
    public static void PosCalcArnav(double r, double a){

        double currentAngle = worldAngle_rad;

        double thetaDelta = currentAngle - lastAngle;


        double wheelRightCurrent = r;
        double wheelAuxCurrent = -a;

        double rightCM = wheelRightCurrent * cmPerTick;
        double auxCM = wheelAuxCurrent * cmPerTick;

        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

        double rightDeltaCM = wheelRightDelta * cmPerTick;
        double auxDeltaCM = wheelAuxDelta * cmPerTick;

        //this doenst make sense my guy arnav
        //double deltaX = auxDeltaCM - (thetaDelta * 7.5);
        double deltaX = auxDeltaCM - (thetaDelta * strafeConstant);
        double deltaY = rightDeltaCM;

        worldXPosition += deltaX;
        worldYPosition += deltaY;


        lastAngle = worldAngle_rad;

        //save the last positions for later
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;

    }
}