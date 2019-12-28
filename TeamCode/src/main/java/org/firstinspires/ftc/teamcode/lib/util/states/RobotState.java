package org.firstinspires.ftc.teamcode.lib.util;

public enum RobotState{

    MOVING_TO_TARGET {
        @Override
        public RobotState advance(){
            return AT_TARGET;
        }
    },
    AT_TARGET {
        @Override
        public RobotState advance(){
            return MOVING_TO_TARGET;
        }
    },
    STOPPED {
        @Override
        public RobotState advance(){
            return MOVING_TO_TARGET;
        }
    };


    public abstract RobotState advance();
    public RobotState stop(){
        return STOPPED;
    }



}
