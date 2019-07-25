package org.firstinspires.ftc.teamcode.opmodes.auto.roadrunner;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
  public static double DISTANCE = 60;

  @Override
  public void runOpMode() throws InterruptedException {
    SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

    Trajectory trajectory = drive.trajectoryBuilder()
        .forward(DISTANCE)
        .build();

    waitForStart();

    if (isStopRequested()) return;

    drive.followTrajectorySync(trajectory);
  }
}