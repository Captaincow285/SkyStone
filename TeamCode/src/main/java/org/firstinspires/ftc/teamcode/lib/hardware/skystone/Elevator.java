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

    private final double TICKS_TO_INCHES = 1;
    private final double INCHES_TO_TICKS = 1;

    /**
     * sets up the Elevator class
     * @param elev representing the hardware location of the motor that controls the Elevator
     */
    public void setup(DcMotor elev){

        elevator = elev;

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDe = new PIDController(eKp, eKi, eKp);

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

        PIDe.setSetpoint(target * INCHES_TO_TICKS);

    }

    @Override
    public void finishJob() {

    }
}
