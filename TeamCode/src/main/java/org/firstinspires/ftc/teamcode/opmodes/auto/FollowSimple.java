package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.hardware.base.RevMotor;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.recording.InputManager;
import org.firstinspires.ftc.teamcode.lib.util.PIDController;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.io.File;

import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.lib.movement.MyPosition.worldYPosition;
import static org.firstinspires.ftc.teamcode.lib.movement.RobotMovement.manualControl;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;

@Autonomous
public class FollowSimple extends OpMode {

    DriveTrain dt = new DriveTrain();

    private RevBulkData revExpansionMasterBulkData;

    private ExpansionHubEx revMaster;

    private RevMotor[] motors;

    int robo = 0;

    PIDController PIDx = new PIDController(0.05, 0, 0);
    PIDController PIDy = new PIDController(0.05, 0, 0);

    double xPower = 0;
    double yPower = 0;

    double xTarget = 0;
    double yTarget = 0;

    InputManager inputManager = new InputManager();

    @Override
    public void init(){

        RevExtensions2.init();

        revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        motors = new RevMotor[]{new RevMotor((ExpansionHubMotor) hardwareMap.get("fl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("fr"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("bl"),true), new RevMotor((ExpansionHubMotor) hardwareMap.get("br"),true)};

        dt.initMotors(motors);

        MyPosition.setPosition(0, 0, 0);

        inputManager.setupPlayback(new File("/sdcard/Download/InputFiles/inputTest.txt"));


        PIDx.setOutputLimits(-1, 1);

        PIDy.setOutputLimits(-1, 1);

    }

    @Override
    public void loop(){

        //ElapsedTime timer = new ElapsedTime();

        getRevBulkData();

        //dt.applyMovement();

        MyPosition.PosCalc2Wheel(
                dt.fr.getCurrentPosition(),
                dt.bl.getCurrentPosition());

        applyMovement();

        /*switch(robo){
            case 0: {
                xTarget = 50;
                yTarget = 30;

                if(PIDx.getError() <= 2 && PIDy.getError() <= 2){
                    robo++;
                }
                break;
            }

            case 1: {
                xTarget = 0;
                yTarget = 0;


            }
        }*/

        for(Point point: inputManager.getPoints()){
            xTarget = point.x;
            yTarget = point.y;

            telemetry.addData("point: ", point);
            telemetry.update();
        }

        telemetry.addLine("wx count: " + worldXPosition);
        telemetry.addLine("wy: " + worldYPosition);
        telemetry.addLine("xTarget: " + xTarget);
        telemetry.addLine("yPower: " + yPower);
        telemetry.update();
    }

    public void applyMovement(){

        movement_x = PIDx.getOutput(worldXPosition, xTarget);
        movement_y = PIDy.getOutput(worldYPosition, yTarget);



    }


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
