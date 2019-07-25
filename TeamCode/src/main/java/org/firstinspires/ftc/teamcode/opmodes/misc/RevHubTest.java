package org.firstinspires.ftc.teamcode.opmodes.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp (group = "test")
public class RevHubTest extends OpMode {

  private ExpansionHubEx revMaster;
  private ExpansionHubEx revSlave;
  private RevBulkData revExpansionMasterBulkData;

  @Override
  public void init() {

    RevExtensions2.init();

    revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");
    revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 5");

    revMaster.setPhoneChargeEnabled(true);
    revMaster.setLedColor(255, 0, 0);


    revSlave.setLedColor(255, 255, 0);
  }

  @Override
  public void loop() {




  }

  private void getRevBulkData() {
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

//        }



  }
}
