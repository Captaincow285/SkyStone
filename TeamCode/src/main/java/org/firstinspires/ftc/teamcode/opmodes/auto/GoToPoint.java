package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Position;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.applyTarget;
import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.setTarget;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

@Autonomous
public class GoToPoint extends Robot {

  ArrayList<Point> points = new ArrayList<>();
  int auto;

  @Override
  public void init(){
    super.init();

    MyPosition.setPosition(0,0,0);

    autoState = AutoStates.START;
    auto = 0;

  }

  @Override
  public void loop(){
    super.loop();

    applyTarget();
/*

    switch (auto){
      case 0: {

        auto++;
        break;
      }

      case 1: {

        setTarget(new Point(30,30));

        break;
      }

      case 2: {

        setTarget(new Point(0,0));

        break;
      }



    }
*/

    switch (autoState){
      case START:{

        autoState = AutoStates.MOVE;
        break;
      }
      case MOVE:{

        setTarget(new Position(30, 30, 90));

        if(roboState == RobotStates.AT_TARGET) {
          autoState = AutoStates.END;
        }
        break;
      }
      case MOVE2:{

        setTarget(new Position(0,0, Math.toRadians(90)));

        if(roboState == RobotStates.AT_TARGET) {
          autoState = AutoStates.MOVE3;
        }
        break;
      }
      case MOVE3:{

        setTarget(new Position(0,30,0));

        if(roboState == RobotStates.AT_TARGET) {
          autoState = AutoStates.MOVE4;
        }
        break;
      }
      case MOVE4:{

        setTarget(new Position(0,0,0));

        if(roboState == RobotStates.AT_TARGET) {
          autoState = AutoStates.END;
        }
        break;
      }
      case END:{

        stop();

      }
    }

  }

}
