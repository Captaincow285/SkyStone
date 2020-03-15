package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;

import static org.firstinspires.ftc.teamcode.lib.movement.Pose.setPose;

@TeleOp (group = "Basic")
public class OneController extends Robot {

    private boolean yButton2Toggle = false;

    private boolean clampToggle =  false;
    private boolean delivModeToggle = false;
    private boolean extended = false;

    private boolean lastB;
    private boolean lastLBumper;
    private boolean lastX;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        super.init();

        isAuto(false);

        setPose(0,0,0);

    }

    @Override
    public void start(){
        setPose(0,0,0);
    }


    @Override
    public void loop() {
        super.loop();

        if(gamepad2.y && timer.seconds() >= 1){
            yButton2Toggle = !yButton2Toggle;
            timer.reset();
        }


        //Don't touch!
        dt.manualControl(gamepad1);

        //Change to multiple-cases intake/eject
        intake.setTarget(gamepad1.left_trigger - gamepad1.right_trigger, gamepad2.left_trigger >= 0.01);


        if (gamepad1.x != lastX)
        {
            delivModeToggle = !delivModeToggle;
        }
        lastX = gamepad1.b;

        if(delivModeToggle){
            intake.setUsingSensor(true);
        }
        else {
            intake.setUsingSensor(false);
        }


        //Switch to toggle with Y
        if(gamepad1.left_bumper){
            fm.setTarget(false);
        } else if(gamepad1.right_bumper){
            fm.setTarget(true);
        }


        if(gamepad1.left_trigger >= .1) {
            elevator.setTarget(gamepad1.right_stick_y);
        }


        if (gamepad1.left_bumper != lastLBumper)
        {
            extended = !extended;
        }
        lastLBumper = gamepad1.b;

        if(extended){
            depositor.setTarget(0.4);
            dt.setSlowmode(0.5);
        } else {
            depositor.setTarget(1);
            dt.setSlowmode(1);
        }


        if (gamepad1.b != lastB)
        {
            clampToggle = !clampToggle;
        }
        lastB = gamepad1.b;

        if(clampToggle){
            clamp.setTarget(GlobalVars.ClawStates.GRIPPING);
        }
        else {
            clamp.setTarget(GlobalVars.ClawStates.IDLE);
        }
    }
}
