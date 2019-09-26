package org.firstinspires.ftc.teamcode.lib.util.Skystone;

import org.firstinspires.ftc.teamcode.lib.util.Skystone.Stone;

public class Quarry{

    private ArrayList<Stone> quarry = new ArrayList<Stone>[6];

    public Quarry(){



    }

    public void populateQuarry(){

        for(int i = 0; i < 6; i++){

            quarry.add(i, new Stone(i));

        }

    }

    public void populateSkystones(int ss_1, int ss_2){

        quarry[ss_1].setIsSkystone(true);
        quarry[ss_2].setIsSkystone(true);

    }

    public int getRoughStonePositon(Stone stone){
        return getRoughtStonePosition(stone.getPosition());
    }

    public int getRoughStonePosition(int position){
        return (position * STONE_LENGTH);
    }

    public ArrayList<Stone> getQuarry(){
        return quarry;
    }


}