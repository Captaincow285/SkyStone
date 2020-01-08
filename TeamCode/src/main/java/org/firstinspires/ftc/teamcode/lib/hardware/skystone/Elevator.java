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
    private double lastTarget = 0;

    private final double TICKS_PER_INCH = 3400;
    private final double INCHES_PER_TICK = 1;

    private boolean elevatorLocked = false;



    /**
     * sets up the Elevator class
     * @param elev representing the hardware location of the motor that controls the Elevator
     */
    public void init(DcMotor elev){

        elevator = elev;

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //elevator.setTargetPosition(0);
        //elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PIDe = new PIDController(eKp, eKi, eKd);

        PIDe.setSetpoint(target);
        //PIDe.setSetpointRange(100);


    }

    @Override
    public void update() {

        elevator.setPower(PIDe.getOutput(elevator.getCurrentPosition()));
        //elevator.setTargetPosition((int)target);
        //elevator.setPower(0.5);
        //elevator.setPower(target);


    }

    /**
     *
     * @param target in inches
     */
    public void setTarget(double target) {

        if(target <= 0){
            target = 0;
        }

        if(elevatorLocked){
            this.target = lastTarget;
        } else {
            this.target = target * TICKS_PER_INCH;

            PIDe.setSetpoint(this.target);

            //lastTarget = this.target;
        }

    }

    public void setElevatorLocked(boolean state){
        elevatorLocked = state;
        lastTarget = elevator.getCurrentPosition();
    }

    public double getTarget(){
        return target;
    }

    @Override
    public void finishJob() {

    }

    public int getCurrentTicks(){
        return elevator.getCurrentPosition();
    }
}
