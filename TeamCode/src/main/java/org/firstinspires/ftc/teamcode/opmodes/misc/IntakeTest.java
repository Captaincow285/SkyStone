package org.firstinspires.ftc.teamcode.opmodes.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.hardware.skystone.Intake;

public class IntakeTest extends OpMode{

    Intake frank = new Intake();

    @Override
    public void init() {

        frank.init(hardwareMap.dcMotor.get("frankLeft"), hardwareMap.dcMotor.get("frankRight"));

    }

    @Override
    public void loop() {

        if(gamepad1.a){
            frank.setTarget(1);
        }

        if(gamepad1.b){
            frank.setTarget(0);
        }


        frank.update();

    }
}
