package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import org.firstinspires.ftc.teamcode.lib.hardware.base.RobotGyro;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;

public class PoseOdo extends RobotGyro {



    @Override
    public void init(){

        super.init();


    }

    @Override
    public void init_loop(){
        super.init_loop();

        Pose.setPose(Pose.zero);

    }

    @Override
    public void start(){
        super.start();

        dt.setTarget(new Pose(30, 30, Math.toRadians(180)));

        telemetry.addLine("target: " + dt.getTargetString());

    }

    @Override
    public void loop(){
        super.loop();

    }


}
