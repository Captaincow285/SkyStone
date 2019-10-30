package org.firstinspires.ftc.teamcode.lib.hardware.base;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.skystone.Intake;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.nio.channels.DatagramChannel;
import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDx;
import static org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain.PIDy;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

/**
 * main robot class
 * all opmodes will extend this class
 */
//@TeleOp
public class Robot extends OpMode{

  private boolean isAuto = true;

  public DecimalFormat df = new DecimalFormat("###.###");

  private RevBulkData revExpansionMasterBulkData;

  private ExpansionHubEx revTx;
  // used in future if you need bulk reads from the other hub
  private ExpansionHubEx revRx;

  private RevMotor[] motors;

  public DriveTrain dt = new DriveTrain();
  public Intake intake = new Intake();

  //public FtcDashboard dashboard = FtcDashboard.getInstance();
  //public TelemetryPacket packet = new TelemetryPacket();



  @Override
  public void init() {

    RevExtensions2.init();

    revTx = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 6");
    revRx = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 9");

    motors = new RevMotor[]{new RevMotor((ExpansionHubMotor) hardwareMap.get("fl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("fr"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("bl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("br"),true)};

    //stores gamepads in global variables
    if(!isAuto){
      getGamepads(gamepad1, gamepad2);
      //dt.initGyro(hardwareMap.get(BNO055IMU.class, "imu"));
    }

    dt.initMotors(motors);

    intake.init(hardwareMap.get(DcMotor.class, "intakeLeft"), hardwareMap.get(DcMotor.class, "intakeRight"));

  }

  @Override
  public void loop() {

    //gets sensor data
    getRevBulkData();

    //if the robot is not finished, apply the motor powers to the motors
    if(roboState != RobotStates.FINISHED) {


    }

    if(isAuto) {
      dt.update();
    } else {
      dt.applyMovement();
    }
    intake.update();

    //fetch our rotation in radians from the imu
    //worldAngle_rad = Double.parseDouble(df.format(AngleWrap(dt.getGyroRotation(AngleUnit.RADIANS))));

    //calculate our x and y coordinates
    Pose.PosCalc(
        dt.fl.getCurrentPosition(),
        dt.fr.getCurrentPosition()
    );

    //update our auto states
    //updateAutoState(); this is currently done inside the opmode instance

    //update our robot states
    //updateAtTargetAlt();

    //telemetry.addLine("positions set!");

    telemetry.addLine("wx: " + worldXPosition);
    telemetry.addLine("wy: " + worldYPosition);
    //telemetry.addLine("wa: " + Math.toDegrees(worldAngle_rad));
    telemetry.addLine("");
   // telemetry.addLine("r: " + dt.fr.getCurrentPosition());
   // telemetry.addLine("a: " + dt.bl.getCurrentPosition());
    //telemetry.addLine("");
    telemetry.addLine("auto state: " + autoStateLZ);
    telemetry.addLine("");
    telemetry.addLine("robot state: " + roboState);
    //telemetry.addLine("strafe const: " + strafeConstant);

    telemetry.update();

   /* packet.put("wx", worldXPosition);
    packet.put("wy", worldYPosition);
    packet.put("wa", Math.toDegrees(worldAngle_rad));
    packet.put("auto", auto);
    packet.put("autostate", autoState);
    packet.put("robot state", roboState);*/

    //dashboard.sendTelemetryPacket(packet);

  }

/*

ROBOT FUNCTIONS

*/



/*

UTIL FUNCTIONS

*/

  /**
   * stores gamepads in global variables
   * @param main main gamepad, used for driving the robot base
   * @param aux auxiliary gamepad, used for driving the scoring mechanisms
   */
  public void getGamepads(Gamepad main, Gamepad aux){

    mainGp = main;
    auxGp = aux;

  }

  /**
   * updates the robot state in relation to whether or not we are at our target position or not
   */
  private void updateAtTarget(){
    if(((worldXPosition >= xTarget -mTolerance) && (worldXPosition <= xTarget+mTolerance)) && ((worldYPosition >= yTarget -mTolerance) && (worldYPosition <= yTarget+mTolerance)) && ((Math.toDegrees(worldAngle_rad) >= aTarget - aTolerance) && (Math.toDegrees(worldAngle_rad) <= aTarget +aTolerance))){
      roboState = RobotStates.AT_TARGET;
      wxRelative = 0;
      wyRelative = 0;
    } else if(roboState == RobotStates.AT_TARGET){
      roboState = RobotStates.MOVING_TO_TARGET;
    }
  }

  /**
  * updates the robot state in relation to the error of the pid loops
  */
  private void updateAtTargetAlt(){

    if(PIDx.getError() < 1 && PIDy.getError() < 1){
      roboState = RobotStates.AT_TARGET;
      wxRelative = 0;
      wyRelative = 0;
    }

  }

  /**
   * updates our auto state in relation to our robot state
   */
  private void updateAutoState(){

    if(roboState == RobotStates.AT_TARGET){

      roboState = RobotStates.STOPPED;
      auto++;

    }
  }

  /**
   * denotes whether or not the opmode currently running is auto or not
   * @param isAuto true if this is auto, false if it is not auto
   */
  public void isAuto(boolean isAuto){
    this.isAuto = isAuto;
  }

  /**
   * Gets all the data from the expansion hub in one command to increase loop times
   */
  public void getRevBulkData() {
//        boolean needToPollMaster = !AutoFeeder.canPollMasterAtLowerRate ||
//            currTimeMillis-lastUpdateMasterTime > 300;
//        if(needToPollMaster){
    RevBulkData newDataMaster;
    try{
      newDataMaster = revTx.getBulkInputData();
      if(newDataMaster != null){
        revExpansionMasterBulkData = newDataMaster;
      }
    }catch(Exception e){
      //don't set anything if we get an exception
    }


    for(RevMotor revMotor : motors) {
      if (revMotor == null) {
        continue;
      }
      if (revExpansionMasterBulkData != null) {
        revMotor.setEncoderReading(revExpansionMasterBulkData.getMotorCurrentPosition(revMotor.myMotor));
      }
    }

  }
}
