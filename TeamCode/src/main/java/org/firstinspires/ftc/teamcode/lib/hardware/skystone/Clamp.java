package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;

//probably add the extra S4T encoder onto this
public class Clamp extends Subsystem {

    GlobalVars.RobotStates clampState = GlobalVars.RobotStates.STOPPED;

    private CRServo clamp;

    //private double target;
    private boolean target;

    private final double MOVE_SPEED = 1;

    //private final double GEAR_REDUCTION = 1;
    //private final double INCHES_TO_TICKS = 1;

    private DigitalChannel touch;

    public void setup(CRServo clamp, DigitalChannel dc){
    //public void setup(CRServo clamp){

        this.clamp = clamp;
        this.touch = dc;

    }

    @Override
    public void update() {

        if(clampState == GlobalVars.RobotStates.MOVING_TO_TARGET){
            clamp.setPower(MOVE_SPEED);
        } else {
            clamp.setPower(0);
            clampState = GlobalVars.RobotStates.AT_TARGET;
        }

        if(touch.getState() == false){
            clampState = GlobalVars.RobotStates.AT_TARGET;
        } else {
            clampState = GlobalVars.RobotStates.MOVING_TO_TARGET;
        }

    }


    public void setTarget(boolean target) {

        this.target = target;
        clampState = GlobalVars.RobotStates.MOVING_TO_TARGET;

    }

    @Override
    public void finishJob() {

    }
}
