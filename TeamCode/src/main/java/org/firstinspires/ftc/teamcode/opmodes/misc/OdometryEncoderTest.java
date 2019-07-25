package org.firstinspires.ftc.teamcode.opmodes.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (group = "test")
public class OdometryEncoderTest extends OpMode {


  AnalogInput encoder;

  DcMotor motor;

  @Override
  public void init() {

    encoder = hardwareMap.get(AnalogInput.class, "encoder");
    motor = hardwareMap.get(DcMotor.class, "motor");

  }

  @Override
  public void loop() {

    telemetry.addLine("analog encoder voltage: " + encoder.getVoltage());
    telemetry.addLine("motor encoder counts: " + motor.getCurrentPosition());
    telemetry.update();

  }

}
