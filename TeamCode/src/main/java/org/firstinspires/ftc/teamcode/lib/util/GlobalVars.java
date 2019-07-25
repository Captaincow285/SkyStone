package org.firstinspires.ftc.teamcode.lib.util;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;

@Config
public class GlobalVars {

    public static double worldXPosition = 0.0;
    public static double worldYPosition = 0.0;
    public static double worldAngle_rad = 0.0;

    public static double movement_x = 0;
    public static double movement_y = 0;
    public static double movement_turn = 0;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double aTarget = 0;

    public static boolean atTarget = false;

    public static int auto = 0;

    public enum AutoStates{
        START, MOVE, MOVE2, MOVE3, MOVE4, END
    }

    public static AutoStates autoState = AutoStates.START;


    public enum RobotStates{
        FINISHED, AT_TARGET, MOVING_TO_TARGET, MOVING, STOPPED
    }

    public static RobotStates roboState = RobotStates.STOPPED;

    public final static double autoMoveSpeed = 0.5;
    public final static double autoTurnSpeed = 0.5;

    public final static double mTolerance = 1;
    public final static double aTolerance = 1;

    public static double strafeConstant = 19.65;

    //PIDx
    public static double xKp = 0.05;
    public static double xKi = 0;
    public static double xKd = 0;

    //PIDy
    public static double yKp = 0.05;
    public static double yKi = 0;
    public static double yKd = 0;

    //PIDa
    public static double aKp = 0.03;
    public static double aKi = 0;
    public static double aKd = 0;

}
