package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;

public class Intake extends Subsystem {

    private DcMotor intake;
    private ColorSensor intakeSensor;
    private Servo hitter;

    private boolean usingSensor, blockIntaked;

    private double leftTarget, rightTarget, target;
    private boolean hitterTarget = false;

    private final double intakeSoftDeliverySpeed = 0.4;

    private final double HITTER_OPEN = 0.2, HITTER_CLOSED = 0.6;



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

    public void init(DcMotor intake, Servo hitter, ColorSensor intakeSensor) {

        this.intake = intake;
        this.intakeSensor = intakeSensor;
        this.hitter = hitter;

        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hitter.setPosition(HITTER_OPEN);


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

        if (hitterTarget) {
            hitter.setPosition(HITTER_OPEN);
        } else {
            hitter.setPosition(HITTER_CLOSED);
        }


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

    /**
     * updates the intakes target power
     * @param target the target power
     */
    public void setTarget(double target, boolean hitterTarget) {

        this.target = target;
        this.hitterTarget = hitterTarget;


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
