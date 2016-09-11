package org.firstinspires.ftc.teamcode.Sensors;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;

public interface SensorObj {
    public String getCurrentState();
    public String getName();
    enum states implements StateName{};

}
