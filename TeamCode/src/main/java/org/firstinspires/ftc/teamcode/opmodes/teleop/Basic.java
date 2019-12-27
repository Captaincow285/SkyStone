package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;

import static org.firstinspires.ftc.teamcode.lib.movement.Pose.setPose;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.strafeConstant;

@TeleOp (group = "Basic")
public class Basic extends Robot {


    @Override
    public void init() {
        super.init();

        isAuto(false);

        setPose(0,0,0);

    }

    @Override
    public void loop() {
        super.loop();

        dt.manualControl(gamepad1);
        intake.setTarget(gamepad1.left_trigger - gamepad1.right_trigger);

        if(gamepad1.left_bumper){
            fm.setTarget(false);
        } else if(gamepad1.right_bumper){
            fm.setTarget(true);
        }

        elevator.setTarget(gamepad2.right_stick_y);
        depositor.setTarget(gamepad2.right_trigger - gamepad2.left_trigger);

        if(gamepad2.right_bumper){
            clamp.setTarget(GlobalVars.ClawStates.GRIPPING);
        } else if(gamepad2.left_bumper){
            clamp.setTarget(GlobalVars.ClawStates.DEPOSITING);
        } else if(gamepad2.right_stick_button){
            clamp.setTarget(GlobalVars.ClawStates.IDLE);
        }



    }

}
