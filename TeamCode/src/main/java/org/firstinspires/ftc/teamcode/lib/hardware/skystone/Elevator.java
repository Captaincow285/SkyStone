package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

/**
 * represents the vertical Elevator on the robot
 */
public class Elevator extends Subsystem {

    private DcMotor elevator;
    private PIDController PIDe;

    private double target = 0;

    private final double TICKS_PER_INCH = 1000;
    private final double INCHES_PER_TICK = 1;

    /**
     * sets up the Elevator class
     * @param elev representing the hardware location of the motor that controls the Elevator
     */
    public void init(DcMotor elev){

        elevator = elev;

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDe = new PIDController(eKp, eKi, eKd);

        PIDe.setSetpoint(target);

    }

    @Override
    public void update() {

        elevator.setPower(PIDe.getOutput(elevator.getCurrentPosition()));

    }

    /**
     *
     * @param target in inches
     */
    public void setTarget(double target) {

        PIDe.setSetpoint(target * TICKS_PER_INCH);

    }

    @Override
    public void finishJob() {

    }

    public int getCurrentTicks(){
        return elevator.getCurrentPosition();
    }
}
