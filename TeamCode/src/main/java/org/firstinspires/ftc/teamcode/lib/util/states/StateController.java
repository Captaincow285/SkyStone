package org.firstinspires.ftc.teamcode.lib.util;

public class StateController {

    public RobotState robo = RobotState.STOPPED;
    public LZAutoState lzAuto = LZAutoState.START;

    public StateController(){

    }

    public void advanceRobo(){
        robo = robo.advance();
    }

    public void advanceLZAuto(){
        lzAuto = lzAuto.advance();
    }

    public void advanceBZAuto(){

    }

    public void stopRobot(){
        robo = robo.stop();
    }

    public void stopAuto(){
        lzAuto = lzAuto.stop();
    }





}
