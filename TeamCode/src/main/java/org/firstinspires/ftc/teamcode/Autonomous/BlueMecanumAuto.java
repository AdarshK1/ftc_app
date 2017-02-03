package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.util.ArrayList;

/**
 * Created by akulkarni on 1/15/17.
 */

@Autonomous(name="Blue Auto Mecanum", group="TeamCode")
//@Disabled
public class BlueMecanumAuto extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor fld;
    DcMotor frd;
    DcMotor bld;
    DcMotor brd;
    OpticalDistanceSensor odsRight;
    OpticalDistanceSensor odsLeft;
    ModernRoboticsI2cRangeSensor range;
    ArrayList<Double> odsVal = new ArrayList<Double>();

    Servo flip;

    CRServoImpl chamber;

    static final int LED_CHANNEL = 5;
    private final int NAVX_DIM_I2C_PORT = 5;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private final double TARGET_ANGLE_DEGREES = -45.0;
    private final double TOLERANCE_DEGREES = 1.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;

    DcMotor shoot;
    DeviceInterfaceModule dim;

//    static final int LED_CHANNEL = 5;

//    float error;

    static final double WHITE_THRESHOLD = 0.99;

    @Override
    public void runOpMode() throws InterruptedException {

        /******** Initialize all devices ********/
//        ods = hardwareMap.opticalDistanceSensor.get("ods");
//        ods.enableLed(true);
//        rangeBL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeBL");
//        rangeBR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeBR");
//        rangeLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");

        /******** Initialize Drive *********/
        brd = hardwareMap.dcMotor.get("brd");
        fld.setDirection(DcMotor.Direction.REVERSE);
        bld = hardwareMap.dcMotor.get("bld");
        brd.setDirection(DcMotor.Direction.REVERSE);

        frd = hardwareMap.dcMotor.get("frd");
        fld.setDirection(DcMotor.Direction.REVERSE);
        fld = hardwareMap.dcMotor.get("fld");
        fld.setDirection(DcMotor.Direction.REVERSE);

        /******** Wait for Start ********/
        while (!isStarted()){
            idle();
        }
//
//        /******** Shoot two balls ********/
//
//
//        /******** Forward until ODS is in line with white line ********/
////        //set the directions to forward so the robot will move forward
////        frd.setDirection(DcMotor.Direction.FORWARD);
////        fld.setDirection(DcMotor.Direction.FORWARD);
////        brd.setDirection(DcMotor.Direction.FORWARD);
////        bld.setDirection(DcMotor.Direction.FORWARD);
//
//        //keep moving forward while the average of the two values is less than 100 cm
////        while(opModeIsActive() && ((rangeBL.getDistance(DistanceUnit.CM) + rangeBR.getDistance(DistanceUnit.CM))/2) < 100){
//            //move forward at .8 power
//            frd.setPower(.8);
//            fld.setPower(.8);
//            brd.setPower(.8);
//            bld.setPower(.8);
////            sleep(5000);
//            //telemetry
////            telemetry.addData("Left Back Range", rangeBL.getDistance(DistanceUnit.CM));
////            telemetry.addData("Right Back Range", rangeBR.getDistance(DistanceUnit.CM));
//            telemetry.update();
//            idle();
//        }
//
//        //after you're 100 cm away from the back wall, stop
////        frd.setPower(0);
////        fld.setPower(0);
////        brd.setPower(0);
////        bld.setPower(0);
//
//        /******** Strafe horizontally towards beacon until both ODS' hit white line ********/
//        //set the directions to left so the robot will move left
//
//        //keep moving left while the average of the two values is less than 10 cm
////        while(opModeIsActive() && (rangeLeft.getDistance(DistanceUnit.CM)) < 10){
//            //move left at .8 power
////            frd.setPower(.8);
////            fld.setPower(-.8);
////            brd.setPower(-.8);
////            bld.setPower(.8);
////            //telemetry
////            telemetry.addData("Range", rangeLeft.getDistance(DistanceUnit.CM));
////            telemetry.update();
////            idle();
//        }
//
//        //after you're 10 cm away from the side wall, stop
////        frd.setPower(0);
////        fld.setPower(0);
////        brd.setPower(0);
////        bld.setPower(0);
////
////        /******** Strafe diagonally 3 tiles until you see the white line ********/
////        fld.setDirection(DcMotor.Direction.REVERSE);
////        brd.setDirection(DcMotor.Direction.REVERSE);
////
////        fld.setPower(1); //totally random power value?
////        brd.setPower(.9); //again, totally rand value?
////        frd.setPower(0);
////        bld.setPower(0);
////        sleep(5000);
////
////        double total = 0;
////        for (Double n :odsVal)
////        {
////            total += n;
////        }
////        double avg = total/(odsVal.size());
////
////        while (opModeIsActive() && (ods.getLightDetected() < avg * 3)){
////            telemetry.addData("Light Level",  ods.getLightDetected());
////            telemetry.update();
////            idle();
////        }
////
////        frd.setPower(0);
////        bld.setPower(0);
////        fld.setPower(0);
////        brd.setPower(0);
////
////        sleep(250);
//
//        /******** Follow the line/use range sensor to align with beacon *******/
//        //two range sensors (certain distance apart)
//        //
//
//        /******** Decide which color to push ********/
//
//        /******** Push the correct side of the beacon ********/
//
//        /******** Strafe back 2 tiles ********/
//
//        /******** Use light/range sensor to ensure aligned with beacon ********/
//
//        /******** Decide which color to push ********/
//
//        /******** Push the correct side of the beacon ********/
//
//        /******** Strafe backwards diagonally 3 tiles (hits cap ball) ********/
//
//        /******** Park on center vortex ********/
//}
    }

}