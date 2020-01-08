package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;

import static org.firstinspires.ftc.teamcode.lib.movement.Pose.setPose;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.strafeConstant;

@TeleOp (group = "Basic")
public class Basic extends Robot {

    private boolean yButton2Toggle = false;

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




        dt.manualControl(gamepad1);



        intake.setTarget(gamepad1.left_trigger - gamepad1.right_trigger);
        if(gamepad1.y){
            intake.setUsingSensor(true);
        } else if(gamepad1.x){
            intake.setUsingSensor(false);
        }

        if(gamepad1.left_bumper){
            fm.setTarget(false);
        } else if(gamepad1.right_bumper){
            fm.setTarget(true);
        }

        elevator.setTarget(gamepad2.right_stick_y);
        elevator.setElevatorLocked(yButton2Toggle);


        if(gamepad2.right_trigger >= 0.01){
            depositor.setTarget(0.4);
        } else {
            depositor.setTarget(1);
        }

        if(gamepad2.right_bumper){
            clamp.setTarget(GlobalVars.ClawStates.GRIPPING);
        } else if(gamepad2.left_bumper){
            clamp.setTarget(GlobalVars.ClawStates.DEPOSITING);
        } else if(gamepad2.right_stick_button){
            clamp.setTarget(GlobalVars.ClawStates.IDLE);
        }




    }

}
