package org.firstinspires.ftc.teamcode.lib.util.Skystone;

import org.firstinspires.ftc.teamcode.lib.util.Skystone.Stone;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.STONE_LENGTH;

public class Quarry{

    private ArrayList<Stone> quarry = new ArrayList<Stone>(6);

    public Quarry(){

        //populateQuarry();

    }

    public void populateQuarry(){

        for(int i = 0; i < 6; i++){

            quarry.add(i, new Stone(i));

        }

    }

    public void populateSkystones(int ss_1, int ss_2){

        quarry.get(ss_1).setIsSkystone(true);
        quarry.get(ss_2).setIsSkystone(true);

    }

    public double getRoughStonePositon(Stone stone){
        return getRoughStonePosition(stone.getPosition());
    }

    public double getRoughStonePosition(int position){
        return (position * STONE_LENGTH);
    }

    public ArrayList<Stone> getQuarry(){
        return quarry;
    }


}