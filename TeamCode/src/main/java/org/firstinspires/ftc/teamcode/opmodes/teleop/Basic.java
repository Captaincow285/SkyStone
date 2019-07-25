package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;

import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.goToPosition;
import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.manualControl;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.strafeConstant;

@TeleOp (group = "Basic")
public class Basic extends Robot {


    @Override
    public void init() {
        super.init();

        MyPosition.setPosition(0,0,0);


    }

    @Override
    public void loop() {
        super.loop();

        manualControl(gamepad1);

        if(gamepad1.dpad_up){
            strafeConstant += 0.1;
        } else if(gamepad1.dpad_down){
            strafeConstant -= 0.1;
        }

        if(gamepad1.left_bumper){
            MyPosition.setPosition(0,0,0);
        }

        if(gamepad1.right_bumper){
            goToPosition(30, 30, 0.5, 1, Math.toRadians(90));
        }


    }

}
