package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class Intake extends Subsystem {

    private DcMotor intake;
    private ColorSensor intakeSensor;

    private boolean usingSensor, blockIntaked;

    private double leftTarget, rightTarget, target;

    private final double intakeSoftDeliverySpeed = 0.4;

    public Intake(){

    }

    public void init(DcMotor intake) {

        this.intake = intake;

        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void init(DcMotor intake, ColorSensor intakeSensor) {

        this.intake = intake;
        this.intakeSensor = intakeSensor;

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

        if(usingSensor){
            if(isBlockIntaked()){
                if(target <= 0){
                    target = 0;
                }
            }

            if(target > 0){
                target = intakeSoftDeliverySpeed;
            }

        }

        intake.setPower(target);


    }

    public void setUsingSensor(boolean isUsingSensor){
        this.usingSensor = isUsingSensor;
    }

    public boolean isBlockIntaked(){
        return getSensorReading() >= 60;
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

    public double getSensorReading(){

        return intakeSensor.green();
    }

    public static void manualControl(Gamepad gamepad){


    }
}
