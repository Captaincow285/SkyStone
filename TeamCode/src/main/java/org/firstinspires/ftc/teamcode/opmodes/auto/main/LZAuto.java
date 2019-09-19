package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.movement.Pose.setPose;
import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.setTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;


@Autonomous (group = "main")
public class LZAuto extends Robot {

  int ss_1_position = 0; //0-6

  @Override
  public void init(){
    super.init();

    //change this to use absolute positions (idk what to base it on) but rn its relative
    setPose((ROBOT_WIDTH/2),0,0);

  }

  @Override
  public void init_loop(){
    //skystone detection

    //super.telemetry.addLine("");

  }

  @Override
  public void loop(){
    super.loop();

    switch(autoStateLZ){

      case START: {

        roboState = RobotStates.STOPPED;

        break;
      }

      case MOVE_TO_SS_1:{

        //target pos is like width calcs for codehs projects
        //modulus?
        /*

        skystone pos is 0 to 5

        treat the back edge of the first stone as (0,0), you move to (0, (ss_pos * length of block) - (robot_length/2))

         */


        if(roboState == RobotStates.STOPPED){
          setTarget(new Point( -(ORIGIN_TO_STONES + (ROBOT_WIDTH/2)),(ss_1_position * STONE_LENGTH) - (ROBOT_LENGTH / 2)));
        } else if(roboState == RobotStates.AT_TARGET){
          roboState = RobotStates.STOPPED;
          autoStateLZ = LZStates.GRAB_SS_1;
        }


        break;

      }

      case GRAB_SS_1: {

        break;
      }

      case MOVE_TO_FOUNDATION:{

      }

      case END: {

        stop();
        break;

      }


    }


  }

}
