package org.firstinspires.ftc.teamcode.SpecificClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Dc Motor object that has all encoder states built in
 * Ramping up and down also in progress
 */


public class DcMotorObj {
    DcMotor dc;

    //TODO decide states

    public DcMotorObj(){}

    public DcMotorObj(DcMotor motor){
        this.dc = motor;
    }

    public void setPower(double power){
        dc.setPower(power);
    }

    public void stop() {
        dc.setPower(0);
    }

    //TODO setup encoders

    //TODO move certain amount

    public void toggleDirection() {

    }

    //TODO encoder methods

    //TODO general setter and getters
}
