package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.hardware.base.RevMotor;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.goToPosition;

@Autonomous
public class PeterPositiongTest extends OpMode {

  DriveTrain dt = new DriveTrain();

  private RevBulkData revExpansionMasterBulkData;

  private ExpansionHubEx revMaster;

  RevMotor[] motors;

  @Override
  public void init() {

    RevExtensions2.init();

    revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

    motors = new RevMotor[]{new RevMotor((ExpansionHubMotor) hardwareMap.get("fl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("fr"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("bl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("br"),true)};

    dt.initMotors(motors);


  }

  @Override
  public void loop() {

    getRevBulkData();

    dt.applyMovement();

    MyPosition.giveMePositions(
        dt.fl.getCurrentPosition(),
        dt.fr.getCurrentPosition(),
        dt.bl.getCurrentPosition());

    goToPosition(30,30, 0.5, 1, Math.toRadians(0));

    telemetry.addLine("wx: " + worldXPosition);
    telemetry.addLine("wy: " + worldYPosition);
    telemetry.addLine("wa: " + worldAngle_rad);
    telemetry.update();

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
      newDataMaster = revMaster.getBulkInputData();
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
