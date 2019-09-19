package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

public class Depositor extends Subsystem {
    
    private DcMotor depositor;

    private PIDController PIDd;

    private double target;

    private final double INCHES_TO_TICKS = 1;

    public void setup(DcMotor depos){

        depositor = depos;

        depositor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        depositor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDd = new PIDController(dKp, dKi, dKp);

        PIDd.setSetpoint(target);

    }
    
    @Override
    public void update() {

        depositor.setPower(PIDd.getOutput(depositor.getCurrentPosition()));

    }

    public void setTarget(double target) {

        PIDd.setSetpoint(target * INCHES_TO_TICKS);

    }

    @Override
    public void finishJob() {

    }
}
