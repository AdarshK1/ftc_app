package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.AvoidXfermode;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;
import org.opencv.core.Range;

/**
 * Created by akulkarni on 1/19/17.
 */

@Autonomous(name="Nav Test Mec", group="TeamCode")
@Disabled
public class NavTestMec extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor fld;
    DcMotor frd;
    DcMotor bld;
    DcMotor brd;
    ModernRoboticsI2cRangeSensor range;

    @Override
    public void runOpMode() throws InterruptedException {

        /******** Initialize Drive *********/
        brd = hardwareMap.dcMotor.get("brd");
        bld = hardwareMap.dcMotor.get("bld");
        frd = hardwareMap.dcMotor.get("frd");
        fld = hardwareMap.dcMotor.get("fld");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        brd.setDirection(DcMotor.Direction.REVERSE);
//        bld.setDirection(DcMotor.Direction.REVERSE);
        frd.setDirection(DcMotor.Direction.REVERSE);
//        fld.setDirection(DcMotor.Direction.REVERSE);

        /******** Wait for Start ********/
        while (!isStarted()){
            idle();
        }

        /******* Strafe left one tile *******/
        while(range.getDistance(DistanceUnit.CM) > 10) {
            frd.setPower(-.7);
            fld.setPower(.7);
            brd.setPower(.7);
            bld.setPower(-.7);
        }
        stopMotors();

    }
    public void stopMotors(){
        frd.setPower(0);
        fld.setPower(0);
        brd.setPower(0);
        bld.setPower(0);
    }
}