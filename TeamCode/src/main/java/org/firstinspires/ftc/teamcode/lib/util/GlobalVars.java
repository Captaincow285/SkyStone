package org.firstinspires.ftc.teamcode.lib.util;

import java.util.ArrayList;

public class GlobalVars {
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

    public final static double mTolerance = 3;
    public final static double aTolerance = 2;

    public static double strafeConstant = 19.65;

}
