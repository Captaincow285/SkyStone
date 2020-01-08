package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;
import org.firstinspires.ftc.teamcode.lib.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.lib.movement.MyPosition;
import org.firstinspires.ftc.teamcode.lib.movement.Point;
import org.firstinspires.ftc.teamcode.lib.movement.Pose;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import org.firstinspires.ftc.teamcode.lib.util.Skystone.Quarry;
import org.firstinspires.ftc.teamcode.lib.util.Skystone.Stone;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.movement.Pose.setPose;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.*;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.LZStates.END;
import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.LZStates.PARK_NEAR;

//@Config
@Autonomous (group = "main")
public class BZAuto extends Robot {

    int ss_1_position = 0; //0-6
    int ss_2_position = 4;

    Quarry quarry = new Quarry();

    //FtcDashboard dashboard = FtcDashboard.getInstance();
    //Telemetry dashboardTelemetry = dashboard.getTelemetry();


    ElapsedTime timer = new ElapsedTime();




    @Override
    public void init(){
        super.init();

        //change this to use absolute positions (idk what to base it on) but rn its relative
        //setPose((ROBOT_WIDTH/2),81.28,0);

        isAuto(true);

        //quarry.populateQuarry();


        timer.reset();
    }

    @Override
    public void init_loop(){
        //skystone detection

        //quarry.populateSkystones(ss_1_position, ss_2_position);

        //super.telemetry.addLine("");

        super.init_loop();

        setPose(0,0, 0);
        fm.setTarget(true);
        autoStateLZ = LZStates.MOVE_TO_FOUNDATION;
    }

    @Override
    public void loop(){
        super.loop();


        switch(autoStateLZ) {

            case START: {

                roboState = RobotStates.STOPPED;
                autoStateLZ = LZStates.MOVE_TO_FOUNDATION;
                fm.setTarget(true);
                roboState = RobotStates.AT_TARGET;
                setPose(0, 0, 0);
                depositor.setTarget(1);


                break;
            }

            case MOVE_TO_FOUNDATION: {


                dt.setTarget(new Point(-38, 71));
                if (Math.abs(-38 - worldXPosition) <= 2 && Math.abs(71 - worldYPosition) <= 2) {
                    autoStateLZ = LZStates.MOVE_FOUNDATION;
                    timer.reset();
                }
                fm.setTarget(true);

                break;
            }

            case MOVE_FOUNDATION: {

            /*

                if(fm.getAtTarget()){
                    if(dt.PIDy.getError() <= 3 && dt.PIDx.getError() <= 3 && roboState == RobotStates.MOVING_TO_TARGET && timer.seconds() >= 2) {

                        roboState = RobotStates.AT_TARGET;
                        autoStateLZ = LZStates.END;
                    }

                    dt.setTarget(new Point(20, 20));
             */

                fm.setTarget(false);

                if(timer.seconds() >= 1) {
                    dt.setTarget(new Point(-38, 3));
                    if (Math.abs(2 - worldYPosition) <= 2) {
                        autoStateLZ = PARK_NEAR;
                    }
                }
                break;

            }



            case PARK_NEAR: {

                fm.setTarget(true);

                dt.setTarget(new Point(91, 1));
                if (Math.abs(91 - worldXPosition) <= 2) {
                    autoStateLZ = END;
                }

            }

            case END: {


                stop();
                break;

            }
        }

        depositor.setTarget(1);


    }

    public void moveToStone(Stone targetStone){

        dt.setTarget(new Point( (ORIGIN_TO_STONES),(quarry.getRoughStonePosition(targetStone.getPosition()))));



    }

    public void collectStone(Stone targetStone){

        intake.setTarget(1);
        dt.setTarget(new Point(worldXPosition, ((ss_1_position) * STONE_LENGTH) - (ROBOT_LENGTH / 2)));

    }


}
