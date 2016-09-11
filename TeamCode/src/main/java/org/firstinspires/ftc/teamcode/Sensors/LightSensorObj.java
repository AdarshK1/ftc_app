package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.LightSensor;

public class LightSensorObj implements SensorObj{

    private LightSensor ls;
    public String currentState;

    double blackRaw;

    enum states{
        CALIBRATING,
        READY,
        ONBLACK,
        ONWHITE,
        ONRED,
        ONBLUE,
        INCONCLUSIVE
    };

    public LightSensorObj(LightSensor ls) {
        this.ls = ls;
        currentState = states.CALIBRATING.toString();
    }

    public void calibrateSensor() {
        enableLED();
        blackRaw = getRawLightDetected();

        //TODO better one
    }

    //TODO Detect color method

    public double getRawLightDetected(){
        return ls.getRawLightDetected();
    }


    public void enableLED() {ls.enableLed(true);}

    public void disableLED() {ls.enableLed(false);}

    public double getLightDetected() {return ls.getLightDetected();}

    public double getRawLightDetectedMax() {return ls.getRawLightDetectedMax();}

    @Override
    public String getCurrentState() {
        return this.currentState;
    }

    @Override
    public String getName() {
        return ls.getDeviceName();
    }
}
