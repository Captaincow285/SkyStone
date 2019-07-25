package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDa;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.movement_turn;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.movement_x;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.movement_y;

@TeleOp(name = "PIDTestGyro", group = "kms")
public class PIDTestGyro extends Robot {

  double kp = 1;

  double setpoint= 0;

  PIDController pid = new PIDController(kp, 0, 0);

  @Override
  public void init() {
    super.init();


  }

  @Override
  public void start() {



  }

  @Override
  public void loop() {

    super.loop();

    movement_turn = pid.getOutput(Math.toDegrees(worldAngle_rad), setpoint);

    if(gamepad1.a){
      setpoint = 0;
    } else if(gamepad1.b){
      setpoint = 90;
    }

    if(gamepad1.dpad_up){
      kp += 0.001;
    } else if(gamepad1.dpad_down){
      kp -= 0.001;
    } else if(gamepad1.dpad_left){
      kp += 0.01;
    } else if(gamepad1.dpad_right){
      kp -= 0.01;
    }

    pid.setP(kp);

    telemetry.addData("kp", pid.P);

    telemetry.update();

  }


  @Override
  public void stop() {

  }
}
