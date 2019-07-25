package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.hardware.base.RevMotor;
import org.firstinspires.ftc.teamcode.opmodes.auto.roadrunner.StandardTrackingWheelLocalizer;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevExtensions2;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class StandardTest extends OpMode {

  private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
  private List<ExpansionHubMotor> motors;

  ExpansionHubEx hub;

  StandardTrackingWheelLocalizer localizer;

  double x, y, a;

  List<Double> wheelPos;
  Pose2d pose2d;

  @Override
  public void init() {

    RevExtensions2.init();

    hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

    leftFront = hardwareMap.get(ExpansionHubMotor.class, "fl");
    leftRear = hardwareMap.get(ExpansionHubMotor.class, "bl");
    rightRear = hardwareMap.get(ExpansionHubMotor.class, "br");
    rightFront = hardwareMap.get(ExpansionHubMotor.class, "fr");

    motors = Arrays.asList(leftFront, rightFront, leftRear, rightRear);

    localizer = new StandardTrackingWheelLocalizer(hub, motors);

  }

  @Override
  public void loop() {

    wheelPos = localizer.getWheelPositions();

    pose2d = localizer.getPoseEstimate();

    localizer.update();


    telemetry.addLine("x: " + pose2d.getX());
    telemetry.addLine("y: " + pose2d.getY());
    telemetry.addLine("a: " + pose2d.getHeading());
    telemetry.update();

  }
}
