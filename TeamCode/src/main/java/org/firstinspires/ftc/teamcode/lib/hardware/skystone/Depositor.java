package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

public class Depositor extends Subsystem {
    
    //private DcMotor depositor;
    private Servo depositor;

    private PIDController PIDd;

    private double target;

    private final double INCHES_TO_TICKS = 1;

    /*public void init(DcMotor depos){

        depositor = depos;

        depositor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        depositor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDd = new PIDController(dKp, dKi, dKd);

        PIDd.setSetpoint(target);

    }*/

    public void init(Servo depos){
        depositor = depos;

        depositor.setPosition(1);
    }
    
    @Override
    public void update() {

        //depositor.setPower(PIDd.getOutput(depositor.getCurrentPosition()));
        depositor.setPosition(target);

    }

    public void setTarget(double target) {

        this.target = target;

    }

    public double getTarget(){
        return target;
    }

    @Override
    public void finishJob() {

    }

    //public int getCurrentTicks(){
      //  return depositor.getCurrentPosition();
    //}
}
