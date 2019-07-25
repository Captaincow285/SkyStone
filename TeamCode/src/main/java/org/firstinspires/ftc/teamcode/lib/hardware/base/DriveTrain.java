package org.firstinspires.ftc.teamcode.lib.hardware.base;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Position;
import org.firstinspires.ftc.teamcode.lib.recording.InputManager;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;
import org.firstinspires.ftc.teamcode.lib.util.Math7571;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import static org.firstinspires.ftc.teamcode.lib.util.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.*;

public class DriveTrain {

  public RevMotor fl, fr, bl, br;
  public BNO055IMU imu;

  private OpMode opMode;
  private HardwareMap hardwareMap;

  private double[] motorPowers = new double[4];

  //the actual speed the robot is moving
  public static double xSpeed = 0;
  public static double ySpeed = 0;
  public static double turnSpeed = 0;

  //last update time
  private long lastUpdateTime = 0;

  public static PIDController PIDx = new PIDController(0.05, 0, 0);
  public static PIDController PIDy = new PIDController(0.05, 0, 0);
  public static PIDController PIDa = new PIDController(0.50, 0, 0);

  double xPower = 0;
  double yPower = 0;

  InputManager inputManager = new InputManager();

  public DriveTrain(){

    //initMotors();

    worldXPosition = 0;
    worldYPosition = 0;
    worldAngle_rad = Math.toRadians(0);

  }

  public void initMotors(RevMotor[] motors) {

    fl = motors[0];
    fr = motors[1];
    bl = motors[2];
    br = motors[3];

    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    bl.setDirection(DcMotorSimple.Direction.REVERSE);

    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  }

  public void initGyro(BNO055IMU IMU){
    imu = IMU;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu.initialize(parameters);



  }

  public void driveInches(double distance){
    System.out.println("distance: " + distance);
  }

  public void drivePID(double distance){

  }

  public void turnGyro(double targetHeading){

  }

  // only for testing purposes
  public void setThrottle(double power){

      fl.setPower(power);
      fr.setPower(power);
      bl.setPower(power);
      br.setPower(power);

  }

  public void strafe(double power){

      fl.setPower(power);
      fr.setPower(-power);
      bl.setPower(-power);
      br.setPower(power);

  }

/* *//**converts movement_y, movement_x, movement_turn into motor powers *//*
  public void applyMovement() {
    long currTime = SystemClock.uptimeMillis();
    if(currTime - lastUpdateTime < 16){
      return;
    }
    lastUpdateTime = currTime;


    double fl_power_raw = movement_y-movement_turn+movement_x*1.5;
    double bl_power_raw = movement_y-movement_turn-movement_x*1.5;
    double br_power_raw = movement_y-movement_turn-movement_x*1.5;
    double fr_power_raw = movement_y-movement_turn+movement_x*1.5;

    //find the maximum of the powers
    double maxRawPower = Math.abs(fl_power_raw);
    if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
    if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
    if(Math.abs(fr_power_raw) > maxRawPower){ maxRawPower = Math.abs(fr_power_raw);}

    //if the maximum is greater than 1, scale all the powers down to preserve the shape
    double scaleDownAmount = 1.0;
    if(maxRawPower > 1.0){
      //when max power is multiplied by this ratio, it will be 1.0, and others less
      scaleDownAmount = 1.0/maxRawPower;
    }
    fl_power_raw *= scaleDownAmount;
    bl_power_raw *= scaleDownAmount;
    br_power_raw *= scaleDownAmount;
    fr_power_raw *= scaleDownAmount;


    //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
    fl.setPower(fl_power_raw);
    bl.setPower(bl_power_raw);
    br.setPower(br_power_raw);
    fr.setPower(fr_power_raw);
  }*/

  public void applyMovement(){

    motorPowers[0] =  movement_y + movement_turn + movement_x;
    motorPowers[1] =  movement_y - movement_turn - movement_x;
    motorPowers[2] =  movement_y + movement_turn - movement_x;
    motorPowers[3] =  movement_y - movement_turn + movement_x;

    /*for(int i = 0; i < motorPowers.length; i++){
      motorPowers[i] = (Math.abs(motorPowers[i]) < 0.075) ? 0:motorPowers[i];
    }*/


    fl.setPower(Range.clip(motorPowers[0], -1, 1));
    fr.setPower(Range.clip(motorPowers[1], -1, 1));
    bl.setPower(Range.clip(motorPowers[2], -1, 1));
    br.setPower(Range.clip(motorPowers[3], -1, 1));


  }

  public void applyMovement(double lx, double ly, double rx){

    double flP, frP, brP, blP;

    flP = ly + rx + lx;
    frP = ly - rx - lx;
    blP = ly + rx - lx;
    brP = ly - rx + lx;

    fl.setPower(Range.clip(flP, -1, 1));
    fr.setPower(Range.clip(frP, -1, 1));
    bl.setPower(Range.clip(blP, -1, 1));
    br.setPower(Range.clip(brP, -1, 1));


  }

  public double getGyroRotation(AngleUnit unit) {
    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit).firstAngle;
  }

}
