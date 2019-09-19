package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;

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



    }

}
