package org.firstinspires.ftc.teamcode.opmodes.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FoundationMoverTest extends OpMode {


    Servo left, right;

    @Override
    public void init() {

        left = hardwareMap.get(Servo.class, "fmLeft");
        right = hardwareMap.get(Servo.class, "fmRight");

    }

    @Override
    public void loop() {

        telemetry.addLine("leftPos: " + left.getPosition() + " rightPos: " + right.getPosition());
        telemetry.update();

    }

}
