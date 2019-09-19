package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class Intake extends Subsystem {

    private DcMotor left, right;

    private double leftTarget, rightTarget, target;

    public Intake(){

    }

    public void init(DcMotor left, DcMotor right) {

        this.left = left;
        this.right = right;

        this.left.setDirection(DcMotorSimple.Direction.REVERSE);

        this.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * updates the intake in relation to its target power
     */
    @Override
    public void update() {

        left.setPower(target);
        right.setPower(target);

    }

    /**
     * updates the intakes target power
     * @param target the target power
     */
    public void setTarget(double target) {

        this.target = target;


    }

    public void setTarget(double leftTarget, double rightTarget){

        this.leftTarget = leftTarget;
        this.rightTarget = rightTarget;

    }

    @Override
    public void finishJob() {

    }
}
