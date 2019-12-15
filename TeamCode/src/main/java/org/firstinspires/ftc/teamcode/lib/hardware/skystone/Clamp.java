package org.firstinspires.ftc.teamcode.lib.hardware.skystone;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.hardware.base.Subsystem;
import org.firstinspires.ftc.teamcode.lib.util.GlobalVars;

//probably add the extra S4T encoder onto this
public class Clamp extends Subsystem {

    GlobalVars.ClawStates clampState = GlobalVars.ClawStates.IDLE;

    private Servo plate, nub;

    private double target;
    //private boolean target;

    private double TARGET_POSITION = 1;

    private final double NUB_CLOSED = 0.45;
    private final double NUB_OPEN = 1;
    private final double PLATE_CLOSED = 0.65;
    private final double PLATE_OPEN = 1;

    //private final double GEAR_REDUCTION = 1;
    //private final double INCHES_TO_TICKS = 1;

    private DigitalChannel touch;

    public void init(Servo plate, Servo nub){
    //public void setup(CRServo clamp){

        this.plate = plate;
        this.nub = nub;

    }

    @Override
    public void update() {

        switch(clampState){
            case IDLE: {
                plate.setPosition(PLATE_CLOSED);
                nub.setPosition(NUB_OPEN);
                break;
            }
            case GRIPPING: {
                plate.setPosition(PLATE_CLOSED);
                nub.setPosition(NUB_CLOSED);
                break;
            }
            case DEPOSITING:{
                plate.setPosition(PLATE_OPEN);
                nub.setPosition(NUB_OPEN);
                break;
            }

        }

    }


    public void setTarget(GlobalVars.ClawStates target) {

        clampState = target;

    }

    @Override
    public void finishJob() {

    }
}
