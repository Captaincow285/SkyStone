package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDa;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

@Config
@TeleOp(name = "PIDTestGyro", group = "kms")
public class PIDTestGyro extends Robot {

  public static double kp = 1;

  public static double setpoint= 0;

  PIDController pid = new PIDController(kp, 0, 0);

  FtcDashboard dashboard = FtcDashboard.getInstance();
  TelemetryPacket packet = new TelemetryPacket();


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

    movement_turn = -pid.getOutput(Math.toDegrees(worldAngle_rad), setpoint);

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



    packet.put("kP", kp);
    packet.put("wa", Math.toDegrees(worldAngle_rad));
    dashboard.sendTelemetryPacket(packet);
  }


  @Override
  public void stop() {

  }
}
