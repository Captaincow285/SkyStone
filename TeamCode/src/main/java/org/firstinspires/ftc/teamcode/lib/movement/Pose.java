package org.firstinspires.ftc.teamcode.lib.movement;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.deltaX;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.deltaY;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.strafeConstant;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.worldYPosition;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.wxRelative;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.wyRelative;

/**
 * represents a Pose(x,y,angle)
 */
public class Pose{

    public static Pose zero = new Pose(0,0,0);

    public static double x, y, a;

    private static double circumfrenceOfWheel = 15.71;//cm
    private static double ticksPerRev = 1440;
    private static double cmPerTick = circumfrenceOfWheel/ticksPerRev;

    //public static double wheelLeftLast = 0.0;
    public static double wheelRightLast = 0.0;
    public static double wheelAuxLast = 0.0;

    public static double lastAngle = 0;

    /**
     *
     * @param x x position
     * @param y y position
     * @param a angle position
     */
    public Pose(double x, double y, double a){

        this.x = x;
        this.y = y;
        this.a = a;

    }

    /**
     * sets x, y, and a to a new Pose
     * @param newPose new Pose(x,y,a) object
     */
    public static void setPose(Pose newPose){
        x = newPose.x;
        y = newPose.y;
        a = newPose.a;
    }

    public static void setPose(Point newPoint){
        x = newPoint.x;
        y = newPoint.y;
    }

    /**
     * sets x, y, and a to a new x, y, and a
     * @param newX new x
     * @param newY new y
     * @param newA new angle
     */
    public static void setPose(double newX, double newY, double newA){
        x = newX;
        y = newY;
        a = newA;

        worldXPosition = x;
        worldYPosition = y;
        worldAngle_rad = a;
    }

    public static void PosCalcRelative(double y, double x){

        double wheelRightCurrent = y;
        double wheelAuxCurrent = -x;

        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

        double rightDeltaCM = wheelRightDelta * cmPerTick;
        double auxDeltaCM = wheelAuxDelta * cmPerTick;

        deltaX = rightDeltaCM;
        deltaY = auxDeltaCM;

        wxRelative += deltaX;
        wyRelative += deltaY;

    }

    public static void PosCalc(double y, double x){

        double wheelRightCurrent = y;
        double wheelAuxCurrent = -x;

        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

        double rightDeltaCM = wheelRightDelta * cmPerTick;
        double auxDeltaCM = wheelAuxDelta * cmPerTick;

        deltaX = rightDeltaCM;
        deltaY = auxDeltaCM;

        worldXPosition += deltaX;
        worldYPosition += deltaY;

        //save the last positions for later
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;

    }

    /**
     * Updates our position on the field using the change from the encoders
     */
    public static void PosCalc(double y, double x, double a){

        double wheelRightCurrent = y;
        double wheelAuxCurrent = -x;

        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

        double rightDeltaCM = wheelRightDelta * cmPerTick;
        double auxDeltaCM = wheelAuxDelta * cmPerTick;

        deltaX = rightDeltaCM;
        deltaY = auxDeltaCM;

        //save the last positions for later
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;

        worldAngle_rad = a;

        double angleIncrement = a - lastAngle;

        if(Math.abs(angleIncrement) > 0){
            //gets the radius of the turn we are in
            double radiusOfMovement = deltaY / angleIncrement;
            //get the radius of our straifing circle
            double radiusOfStraif = deltaX/angleIncrement;





            deltaY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStraif * (1 - Math.cos(angleIncrement)));

            deltaX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStraif * Math.sin(angleIncrement));

        }



        worldXPosition += (Math.cos(lastAngle) * deltaY) + (Math.sin(lastAngle) *
                deltaX);
        worldYPosition += (Math.sin(lastAngle) * deltaY) - (Math.cos(lastAngle) *
                deltaX);


        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;


        lastAngle = a;
    }

}