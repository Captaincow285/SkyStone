package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

//@Config
@Autonomous
public class GoToPoint extends Robot {

  ArrayList<Pose> points = new ArrayList<>();
 // int auto;

  @Override
  public void init(){
    super.init();

    Pose.setPose(new Pose(0,0,0));

    autoState = AutoStates.START;
    auto = 0;
    roboState = RobotStates.MOVING_TO_TARGET;

  }

  @Override
  public void start(){
    auto = 0;
  }

  @Override
  public void loop(){
    super.loop();

    //applyTarget();


    switch (auto){
      case 0: {

        auto++;
        break;
      }

      case 1: {

        dt.setTarget(new Pose(20,20, 0));


        break;
      }

      case 2: {

        dt.setTarget(new Pose(0,0, 0));

        break;
      }



    }


   /* switch (autoState){
      case START:{

        autoState = AutoStates.MOVE;
        break;
      }
      case MOVE:{

        setTarget(new Pose(0, 0, 90));

        if(roboState == RobotStates.AT_TARGET) {
          //autoState = AutoStates.END;
        }
        break;
      }
      case MOVE2:{

        setTarget(new Pose(30,30,90));

        if(roboState == RobotStates.AT_TARGET) {
          //autoState = AutoStates.MOVE3;
        }
        break;
      }
      case MOVE3:{

        setTarget(new Pose(0,30,0));

        if(roboState == RobotStates.AT_TARGET) {
          //autoState = AutoStates.MOVE4;
        }
        break;
      }
      case MOVE4:{

        setTarget(new Pose(0,0,0));

        if(roboState == RobotStates.AT_TARGET) {
          //autoState = AutoStates.END;
        }
        break;
      }
      case END:{

        stop();

      }
    }*/

  }

}
