package org.firstinspires.ftc.teamcode.lib.movement;



import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDa;
import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDx;
import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDy;
import static org.firstinspires.ftc.teamcode.lib.util.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.lib.util.MathFunctions.lineCircleIntersection;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.*;


public class RobotMovement {

    public static boolean followCurve(ArrayList<CurvePoint> allPoints, double followAngle){


        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);


        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle,
                followMe.turnSpeed);

        return true;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> points, Point robotPos, double followRadius){
        CurvePoint followMe = new CurvePoint(points.get(0));

        for(int i = 0; i < points.size()-1; i ++){
            CurvePoint startLine = points.get(i);
            CurvePoint endLine = points.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotPos, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;

            for(Point thisInter : intersections){
                double angle = Math.atan2(thisInter.y - worldYPosition, thisInter.x - worldXPosition);
                double deltaAngle = Math.abs(AngleWrap(angle - worldAngle_rad));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisInter);
                }
            }


        }

        return followMe;

    }

    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double turnSpeed, double preferredAngle){

        double distToTarget = Math.hypot(x-worldXPosition, y- worldYPosition);

        double absoluteAngleToTarget  = Math.atan2(y-worldYPosition, x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYPoint = Math.sin(relativeAngleToPoint) * distToTarget;

        double movementXPower = relativeXPoint / (Math.abs(relativeXPoint) + Math.abs(relativeYPoint));
        double movementYPower = relativeYPoint / (Math.abs(relativeXPoint) + Math.abs(relativeYPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;


        if(distToTarget < 10){
            movement_turn = 0;
        } else {
            movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;
        }

    }


    public static void manualControl(Gamepad gamepad){

        movement_x = Range.clip(gamepad.left_stick_x, -1, 1);
        movement_y = Range.clip(gamepad.left_stick_y, -1, 1);
        movement_turn = Range.clip(gamepad.right_stick_x, -1, 1);

    }

    public static void applyTarget(){

        movement_x = PIDx.getOutput(worldXPosition, xTarget);
        movement_y = PIDy.getOutput(worldYPosition, yTarget);
        movement_turn = -PIDa.getOutput(worldAngle_rad, aTarget);

    }

    public static void setTarget(Point point){

        xTarget = point.x;
        yTarget = point.y;

    }

    public static void setTarget(Point point, double angle){

        xTarget = point.x;
        yTarget = point.y;
        aTarget = Math.toRadians(angle);

    }

    public static void setTarget(Position position){

        xTarget = position.getX();
        yTarget = position.getY();
        aTarget = Math.toRadians(position.getA());

    }

    /**
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange  The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed   The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed   The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     */
    public void gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0)
                gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            movement_turn = (minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        } else {
            movement_turn = 0;
        }
    }

    public static void waitForTarget(){

    }


}
