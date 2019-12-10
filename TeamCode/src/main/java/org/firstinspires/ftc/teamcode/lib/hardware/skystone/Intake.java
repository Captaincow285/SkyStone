package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class Intake extends Subsystem {

    private DcMotor intake;

    private double leftTarget, rightTarget, target;

    public Intake(){

    }

    public void init(DcMotor intake) {

        this.intake = intake;

        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * updates the intake in relation to its target power
     */
    @Override
    public void update() {

        intake.setPower(target);

    }

    public void manaualControl(Gamepad gamepad){

        setTarget(gamepad.right_stick_y);

    }



    /**
     * updates the intakes target power
     * @param target the target power
     */
    public void setTarget(double target) {

        this.target = target;


    }

    @Override
    public void finishJob() {

    }

    public static void manualControl(Gamepad gamepad){


    }
}
