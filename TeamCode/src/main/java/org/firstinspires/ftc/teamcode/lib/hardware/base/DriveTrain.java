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
import org.firstinspires.ftc.teamcode.lib.movement.Pose;
import org.firstinspires.ftc.teamcode.lib.recording.InputManager;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

/**
 * represents the robot's drivebase
 */
public class DriveTrain{

  public RevMotor fl, fr, bl, br;
  public BNO055IMU imu;

  private double[] motorPowers = new double[4];

  //the actual speed the robot is moving
  public static double xSpeed = 0;
  public static double ySpeed = 0;
  public static double turnSpeed = 0;

  //last update time
  private long lastUpdateTime = 0;

  //pid controller objects
  public static PIDController PIDx = new PIDController(xKp, xKi, xKd);
  public static PIDController PIDy = new PIDController(yKp, yKi, yKd);
  public static PIDController PIDa = new PIDController(aKp, aKi, aKd);

  InputManager inputManager = new InputManager();

  public DriveTrain(){

    //initMotors();

    worldXPosition = 0;
    worldYPosition = 0;
    worldAngle_rad = Math.toRadians(0);

  }

  /**
   * initializes the motor objects and sets some necessary details
   * @param motors RevMotor array of the motors
   *               fl = 0
   *               fr = 1
   *               bl = 2
   *               br = 3
   */
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

  /**
   * initializes the imu objects
   * @param IMU imu objects
   */
  public void initGyro(BNO055IMU IMU){
    imu = IMU;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu.initialize(parameters);



  }


  /**
   * sets the forward power for all 4 motors
   * (used only for testing)
   * @param power
   */
  public void setThrottle(double power){

      fl.setPower(power);
      fr.setPower(power);
      bl.setPower(power);
      br.setPower(power);

  }

  /**
   * sets the strafe power for all 4 motors
   * (used only for testing)
   * @param power
   */
  public void strafe(double power){

      fl.setPower(power);
      fr.setPower(-power);
      bl.setPower(-power);
      br.setPower(power);

  }

  /**
   * applies the movement vars to the motors
   */
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

  /**
   * compares the current positions to their targets and calculates the movements for each movement direction/type
   */
  public void update(){

    movement_x = PIDx.getOutput(worldXPosition, xTarget);
    movement_y = PIDy.getOutput(worldYPosition, yTarget);
    movement_turn = -PIDa.getOutput(Math.toDegrees(worldAngle_rad), aTarget);

    applyMovement();

  }

  /**
   * used to designate a new Point(x,y) target for the robot to follow
   * @param point Point(x,y) target
   */
  public void setTarget(Point point){

    xTarget = point.x;
    yTarget = point.y;

    roboState = RobotStates.MOVING_TO_TARGET;

  }

  /**
   * used to designate a new Point(x,y) target for the robot to follow
   * @param point Point(x,y) target
   * @param angle target angle in double
   */
  public void setTarget(Point point, double angle){

    xTarget = point.x;
    yTarget = point.y;
    aTarget = angle;

    roboState = RobotStates.MOVING_TO_TARGET;

  }

  /**
   * used to designate a new Pose(x,y,a) target for the robot to follow
   * @param pose Pose(x, y, a) target
   */
  public void setTarget(Pose pose){

    xTarget = pose.x;
    yTarget = pose.y;
    aTarget = pose.a;

    roboState = RobotStates.MOVING_TO_TARGET;

  }

  /**
   * gets the rotation of the imu
   * @param unit angle unit, degrees or radians
   * @return rotation in unit as a double
   */
  public double getGyroRotation(AngleUnit unit) {
    return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, unit).firstAngle;
  }

}
