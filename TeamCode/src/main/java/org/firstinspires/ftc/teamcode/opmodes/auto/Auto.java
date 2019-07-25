package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.opmodes.auto.Auto.AutoStates.*;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.*;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;


@Autonomous (group = "main")
public class Auto extends Robot {

  public enum AutoStates{
    START, END, MOVE1, MOVE2
  }

  AutoStates robo = START;
  ArrayList<CurvePoint> allPoints;

  @Override
  public void init(){
    super.init();


  }

  @Override
  public void loop(){
    super.loop();

    switch(robo){

      case START: {

        MyPosition.setPosition(0, 0, 0);

        allPoints = new ArrayList<>();

        allPoints.add(new CurvePoint(20.0,  0.0, 25,
            Math.toRadians(90),0.6));

        allPoints.add(new CurvePoint(20.0,  20.0, 25,
            Math.toRadians(90),0.6));

        allPoints.add(new CurvePoint(0.0,  20.0, 25,
            Math.toRadians(90),0.6));

        allPoints.add(new CurvePoint(0.0,  0.0, 25,
            Math.toRadians(90),0.6));

        allPoints.add(new CurvePoint(0.0,  0.0, 25,
            Math.toRadians(135),0.6));

        robo = MOVE1;
        break;
      }

      case MOVE1: {

        followCurve(allPoints, Math.toRadians(90));

        robo = END;
        break;

      }

      case MOVE2:{

        robo = END;
        break;
      }

      case END: {

        Auto.super.stop();
        stop();
        break;

      }


    }


  }

}
