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

@Config
@Autonomous (group = "main")
public class BZAuto extends Robot {

    int ss_1_position = 0; //0-6
    int ss_2_position = 4;

    Quarry quarry = new Quarry();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();




    @Override
    public void init(){
        super.init();

        //change this to use absolute positions (idk what to base it on) but rn its relative
        //setPose((ROBOT_WIDTH/2),81.28,0);

        isAuto(true);

        //quarry.populateQuarry();


    }

    @Override
    public void init_loop(){
        //skystone detection

        //quarry.populateSkystones(ss_1_position, ss_2_position);

        //super.telemetry.addLine("");

        super.init_loop();

        setPose(0,0, 0);
        fm.setTarget(true);
    }

    @Override
    public void loop(){
        super.loop();


        switch(autoStateLZ){

            case START: {

                roboState = RobotStates.STOPPED;
                autoStateLZ = LZStates.MOVE_TO_FOUNDATION;
                fm.setTarget(true);
                //roboState = RobotStates.MOVING_TO_TARGET;



                break;
            }

            case MOVE_TO_FOUNDATION:{

                dt.setTarget(new Pose(0, 80, 0));

                if(roboState == RobotStates.AT_TARGET){

                    fm.setTarget(false);
                    autoStateLZ = LZStates.END;
                }

                break;
            }

            case END: {

                fm.setTarget(false);

                //stop();
                break;

            }


        }

        telemetry.addLine("stone position: " + quarry.getRoughStonePosition(4));


    }

    public void moveToStone(Stone targetStone){

        dt.setTarget(new Point( (ORIGIN_TO_STONES),(quarry.getRoughStonePosition(targetStone.getPosition()))));



    }

    public void collectStone(Stone targetStone){

        intake.setTarget(1);
        dt.setTarget(new Point(worldXPosition, ((ss_1_position) * STONE_LENGTH) - (ROBOT_LENGTH / 2)));

    }


}
