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
    private boolean foundMove = false;

    private boolean lastB;
    private boolean lastRBumper;
    private boolean lastX;
    private boolean lastY;

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

        //Intake
        if(gamepad1.right_trigger >= .01) {
            if(gamepad1.a)
            {
                intake.setTarget((gamepad1.right_trigger * -1), gamepad2.left_trigger >= 0.01);
            }
            else
            {
                intake.setTarget(gamepad1.right_trigger, gamepad2.left_trigger >= 0.01);
            }
        }

        //Delivery Mode toggle
        if (gamepad1.x != lastX)
        {
            delivModeToggle = !delivModeToggle;
        }
        lastX = gamepad1.x;

        if(delivModeToggle){
            intake.setUsingSensor(true);
        }
        else {
            intake.setUsingSensor(false);
        }


        //Foundation Movers
        if (gamepad1.y != lastY)
        {
            foundMove = !foundMove;
        }
        lastY = gamepad1.y;

        if(gamepad1.left_bumper){
            fm.setTarget(false);
        } else if(gamepad1.right_bumper){
            fm.setTarget(true);
        }

        //Linear Slides
        if(gamepad1.left_trigger >= .1) {
            elevator.setTarget(gamepad1.right_stick_y);
        }

        //Clamp extension
        if (gamepad1.right_bumper != lastRBumper)
        {
            extended = !extended;
        }
        lastRBumper = gamepad1.right_bumper;

        if(extended){
            depositor.setTarget(0.4);
            dt.setSlowmode(0.5);
        } else {
            depositor.setTarget(1);
            dt.setSlowmode(1);
        }

        //Stone Clamp
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
