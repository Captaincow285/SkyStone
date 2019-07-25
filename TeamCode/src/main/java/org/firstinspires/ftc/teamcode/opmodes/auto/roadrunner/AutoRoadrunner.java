package org.firstinspires.ftc.teamcode.opmodes.auto.roadrunner;

import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutoRoadrunner extends OpMode {

    SampleMecanumDriveREVOptimized dt;

    @Override
    public void init() {

        dt = new SampleMecanumDriveREVOptimized(hardwareMap);


    }

    @Override
    public void loop() {




        telemetry.addLine("heading: " + dt.getRawExternalHeading());
        telemetry.addLine("wheel pos: " + dt.getWheelPositions());

    }
}
