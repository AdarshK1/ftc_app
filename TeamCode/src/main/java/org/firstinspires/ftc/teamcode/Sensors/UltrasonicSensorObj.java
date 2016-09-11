package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class UltrasonicSensorObj implements SensorObj{

    UltrasonicSensor ultra;

    String currentState;

    enum states {
        READY,
        READING,
        MAXED,

        //TODO finalize states
    };


    public UltrasonicSensorObj(){}

    public UltrasonicSensorObj(UltrasonicSensor ultra){
        this.ultra = ultra;
        this.currentState = states.READY.toString();
    }

    @Override
    public String getName(){
        return ultra.getDeviceName();
    }

    @Override
    public String getCurrentState(){
        return this.currentState;
    }
}
