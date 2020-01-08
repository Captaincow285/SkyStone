package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class FoundationMover extends Subsystem {

    private Servo left, right;

    private double targetPosition = 0.0;

    public FoundationMover(){

    }

    public void init(Servo left, Servo right){
        this.left = left;
        this.right = right;


    }

    @Override
    public void update() {
        left.setPosition(targetPosition);
        right.setPosition(1 - targetPosition);
    }

    public void setTarget(boolean isUp){
        targetPosition = isUp ? .65 : 0;
    }

    @Override
    public void finishJob() {

    }

    public boolean getAtTarget(){
        return left.getPosition() == targetPosition;

    }


}
