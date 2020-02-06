package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import org.firstinspires.ftc.teamcode.lib.util.Skystone.Quarry;
import org.firstinspires.ftc.teamcode.lib.util.Skystone.Stone;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDx;
import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDy;
import static org.firstinspires.ftc.teamcode.lib.movement.Pose.setPose;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

@Config
@Autonomous (group = "main")
public class LZAuto extends Robot {

  int ss_1_position = 0; //0-6
  int ss_2_position = 4;

  Quarry quarry = new Quarry();

  boolean isRed = true, parkClose = true;

  private int directionSwitch = 1;

  ElapsedTime timer = new ElapsedTime();


  @Override
  public void init(){
    super.init();

    //change this to use absolute positions (idk what to base it on) but rn its relative
   // setPose((ROBOT_WIDTH/2),81.28,0);

    isAuto(true);

    quarry.populateQuarry();

    setPose(0,0, 0);

    autoStateLZ = LZStates.START;

    dt.setMaxMotorPowerAuto(0.45);

  }

  @Override
  public void init_loop(){
    //skystone detection

    //quarry.populateSkystones(ss_1_position, ss_2_position);

    //super.telemetry.addLine("");

    super.init_loop();

    setPose(0,0, 0);

    intake.setUsingSensor(true);

    if(gamepad1.left_bumper){
        isRed = true;
    } else if(gamepad1.right_bumper){
        isRed = false;
    }

    if(gamepad1.left_trigger >= 0.01){
        parkClose = true;
    } else if(gamepad1.right_trigger >= 0.01){
        parkClose = false;
    }


  }

  @Override
  public void loop(){
    super.loop();


    switch(autoStateLZ){

      case START: {

        roboState = RobotStates.STOPPED;
        autoStateLZ = LZStates.MOVE_TO_SS_1;
          setPose(0,0, 0);
          //dt.setSlowmode(0.3);

          directionSwitch = isRed ? -1 : 1;
          timer.reset();


        break;
      }

      case MOVE_TO_SS_1:{

        //target pos is like width calcs for codehs projects
        //modulus?
        /*

        skystone pos is 0 to 5

        treat the center of the back edge of the first stone as (0,0), you move to (0, (ss_pos * length of block) - (robot_length/2))

         */


        dt.setTarget(new Point(3 * directionSwitch, 60));
        if((Math.abs(60 - worldYPosition)<= 2) || timer.seconds() >= 3){
            autoStateLZ = LZStates.GRAB_SS_1;

            break;
            //dt.setSlowmode(0.5);
        }

        break;
      }

      case GRAB_SS_1: {

          intake.setTarget(-1);
          dt.setTarget(new Point( directionSwitch * 90, 53));

        if(Math.abs((directionSwitch * 90) - worldXPosition)<= 2 && Math.abs(53 - worldYPosition)<= 2){


              autoStateLZ = LZStates.MOVE_TO_SS_2;
              break;
        }



        break;
      }

        case MOVE_TO_SS_2:{

            dt.setTarget(new Point(directionSwitch * 90, 20));

            if(intake.isBlockIntaked() || Math.abs(20 - worldYPosition)<= 2){



                autoStateLZ = LZStates.GRAB_SS_2;
                break;
            }
            break;

        }

        case GRAB_SS_2:{

          //if(Math.abs(50 - worldXPosition)<= 2){
            //  dt.setTarget(new Point(50, 50));
          //}

          dt.setTarget(new Pose(directionSwitch * 0, 30, 0));
          if((Math.abs(directionSwitch * 0 - worldXPosition)<= 2 && Math.abs(30 - worldYPosition)<= 2)){
              autoStateLZ = LZStates.MOVE_TO_FOUNDATION;
          }

        break;
      }

        case MOVE_TO_FOUNDATION:{

            dt.setTarget(new Point(directionSwitch * 3, -125));
            if((Math.abs((directionSwitch * 3) - worldXPosition)<= 2 && Math.abs(-125 - worldYPosition)<= 2)){
                intake.setTarget(1);
                autoStateLZ = LZStates.MOVE_TO_SS_1;
                timer.reset();
            }

            break;
        }

      case END: {

        stop();
        break;

      }


    }

    //telemetry.addLine("stone position: " + quarry.getRoughStonePosition(4));


  }

  public void moveToStone(Stone targetStone){

      dt.setTarget(new Point( (ORIGIN_TO_STONES),(quarry.getRoughStonePosition(targetStone.getPosition()))));



  }

  public void collectStone(Stone targetStone){

        intake.setTarget(1);
        dt.setTarget(new Point(worldXPosition, ((ss_1_position) * STONE_LENGTH) - (ROBOT_LENGTH / 2)));

    }


}
