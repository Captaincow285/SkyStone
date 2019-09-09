package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;
import org.firstinspires.ftc.teamcode.lib.movement.RobotMovement;
import org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.*;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.setTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.auto;
import static org.firstinspires.ftc.teamcode.opmodes.auto.main.Auto.AutoStates.*;


@Autonomous (group = "main")
public class Auto extends Robot {

  public enum AutoStates{
    START, END, MOVE1, MOVE2
  }

  AutoStates robo = START;


  @Override
  public void init(){
    super.init();

    Pose.setPose(0, 0, 0);

  }

  @Override
  public void loop(){
    super.loop();

    switch(auto){

      case 0 : {

        setTarget(new Point(0,0));

        break;
      }

      case 1: {

        setTarget(new Point(10,10));

        break;

      }

      case 2:{

        setTarget(new Point(10, 20));

        break;
      }

      case 3: {

        Auto.super.stop();
        stop();
        break;

      }


    }


  }

}
