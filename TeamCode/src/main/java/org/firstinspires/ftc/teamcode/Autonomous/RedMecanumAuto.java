package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name="Red Auto Mecanum", group="TeamCode")
//@Disabled
public class RedMecanumAuto extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor fld;
    DcMotor frd;
    DcMotor bld;
    DcMotor brd;

    DcMotor shoot;
    DeviceInterfaceModule dim;

    Servo flip;
    Servo leftBeacon;
    CRServoImpl chamber;

    ColorSensor sensorRGB;
    OpticalDistanceSensor odsRight;
    OpticalDistanceSensor odsLeft;

    ModernRoboticsI2cRangeSensor leftRange;
    ModernRoboticsI2cRangeSensor rightRange;

    ArrayList<Double> odsVal = new ArrayList<Double>();

    static final int LED_CHANNEL = 5;
    private final int NAVX_DIM_I2C_PORT = 5;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

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



    @Override
    public void runOpMode() throws InterruptedException {

        /******** Initialize all devices ********/
        shoot = hardwareMap.dcMotor.get("shoot");

        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION);

        chamber =(CRServoImpl) hardwareMap.crservo.get("chamber");

        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");

        leftBeacon = hardwareMap.servo.get("leftBeacon");
        leftBeacon.setPosition(Servo.MIN_POSITION);

        boolean bLedOn = true;

        dim = hardwareMap.deviceInterfaceModule.get("dim");
        dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("color");
        dim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        navx_device.zeroYaw();

        /************ Create a PID Controller which uses the Yaw Angle as input ************/
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(0);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        odsLeft = hardwareMap.opticalDistanceSensor.get("odsLeft");
        odsRight = hardwareMap.opticalDistanceSensor.get("odsRight");
        odsLeft.enableLed(true);
        odsRight.enableLed(true);

        /******** Initialize Drive *********/
        brd = hardwareMap.dcMotor.get("brd");
        bld = hardwareMap.dcMotor.get("bld");
        frd = hardwareMap.dcMotor.get("frd");
        fld = hardwareMap.dcMotor.get("fld");
//        motorArray.add(fld);
//        motorArray.add(bld);
//        motorArray.add(frd);
//        motorArray.add(brd);


        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());

        bld.setDirection(DcMotor.Direction.REVERSE);
        fld.setDirection(DcMotor.Direction.REVERSE);

        /******* Reset Encoders ********/
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        bld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        bld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        navx_device.zeroYaw();

        /******** Wait for Start ********/
        while (!isStarted()){
            idle();
        }

        /******** Drive forward to shoot position *******/
        encoderDrive(DRIVE_SPEED / 2, 12, 12, 10, true);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());
        sleep(250);
//
//        /******* Flip out cap ball thing *******/
//        flip.setPosition(Servo.MIN_POSITION);
//        sleep(5000);

        /******** Shoot two balls ********/
        shooter(10);
        chamber.setPower(1);
        sleep(1200);
        shooter(10);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());

        /******** Keep going forward to align ********/
        encoderDrive(DRIVE_SPEED / 2, 25, 25, 10, true);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());
        sleep(250);

        /******** Strafe right until 25 cm away ********/
        frd.setPower(DRIVE_SPEED);
        fld.setPower(-DRIVE_SPEED);
        brd.setPower(-DRIVE_SPEED);
        bld.setPower(DRIVE_SPEED);

        while (leftRange.getDistance(DistanceUnit.CM) > 25){
            telemetry.addData("Range Sensor: ", leftRange.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(50);
        }

        stopMotors();

        /******** Align to first white line *******/

        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());
//
        double total = 0;
        for (Double n :odsVal)
        {
            total += n;
        }
        double avg = total/(odsVal.size());
//
        double whiteThreshold = avg * 3;
        boolean rightHit = false;
        boolean leftHit = false;
//
        frd.setPower(DRIVE_SPEED / 6);
        fld.setPower(DRIVE_SPEED / 6);
        brd.setPower(DRIVE_SPEED / 6);
        bld.setPower(DRIVE_SPEED / 6);

        while ((!rightHit && !leftHit) || (!rightHit && leftHit) || (rightHit && !leftHit)){

            telemetry.addData("Left ODS: ", odsLeft.getLightDetected());
            telemetry.addData("Right ODS: ", odsRight.getLightDetected());
            telemetry.update();

            if (odsRight.getLightDetected() > (3 * avg) && rightHit != true){
                frd.setPower(0);
                brd.setPower(0);
                rightHit = true;
            }
            if (odsLeft.getLightDetected() > (3 * avg) && leftHit != true){
                fld.setPower(0);
                bld.setPower(0);
                leftHit = true;
            }
        }

        stopMotors();

        /******** Align using Gyro for Beacon Pushing ********/

        double curr_yaw = navx_device.getYaw();

        telemetry.addData("Current Yaw: ", curr_yaw);
        telemetry.update();

        while (Math.abs(curr_yaw) > .5){
            if (curr_yaw > 0){
                frd.setPower(DRIVE_SPEED / 6);
                brd.setPower(DRIVE_SPEED / 6);
                fld.setPower(0);
                bld.setPower(0);
            } else if (curr_yaw < 0) {
                fld.setPower(DRIVE_SPEED / 6);
                bld.setPower(DRIVE_SPEED / 6);
                frd.setPower(0);
                brd.setPower(0);
            }
            curr_yaw = navx_device.getYaw();
        }


        stopMotors();
        sleep(1000);

        /******** Push the beacon *******/

        /******** Forward and then to the vicinity of the second line  *******/
        frd.setPower(DRIVE_SPEED / 4);
        fld.setPower(DRIVE_SPEED / 4);
        brd.setPower(DRIVE_SPEED / 4);
        bld.setPower(DRIVE_SPEED / 4);
        sleep(250);

        encoderDrive(DRIVE_SPEED, 40, 40, 8, true);


        /******* Align to the second white line *********/
        frd.setPower(DRIVE_SPEED / 6);
        fld.setPower(DRIVE_SPEED / 6);
        brd.setPower(DRIVE_SPEED / 6);
        bld.setPower(DRIVE_SPEED / 6);
        rightHit = false;
        leftHit = false;

        while ((!rightHit && !leftHit) || (!rightHit && leftHit) || (rightHit && !leftHit)){

            telemetry.addData("Left ODS: ", odsLeft.getLightDetected());
            telemetry.addData("Right ODS: ", odsRight.getLightDetected());
            telemetry.update();

            if (odsRight.getLightDetected() > (3 * avg) && rightHit != true){
                frd.setPower(0);
                brd.setPower(0);
                rightHit = true;
            }
            if (odsLeft.getLightDetected() > (3 * avg) && leftHit != true){
                fld.setPower(0);
                bld.setPower(0);
                leftHit = true;
            }
        }

        stopMotors();

        /******** Align using Gyro for Beacon Pushing ********/

        curr_yaw = navx_device.getYaw();

        telemetry.addData("Current Yaw: ", curr_yaw);
        telemetry.update();

        while (Math.abs(curr_yaw) > .5){
            if (curr_yaw > 0){
                frd.setPower(DRIVE_SPEED / 6);
                brd.setPower(DRIVE_SPEED / 6);
                fld.setPower(0);
                bld.setPower(0);
            } else if (curr_yaw < 0) {
                fld.setPower(DRIVE_SPEED / 6);
                bld.setPower(DRIVE_SPEED / 6);
                frd.setPower(0);
                brd.setPower(0);
            }
            curr_yaw = navx_device.getYaw();
        }
        stopMotors();
        sleep(1000);

        /******** Push the beacon *******/

        /******** Move forward so we get at the right angle to hit cap ball ********/
        encoderDrive(DRIVE_SPEED,6,6,5, true);

        /********** Turn to face the cap ball/center vortex********/
        navx_device.zeroYaw();
        curr_yaw = navx_device.getYaw();
        while (Math.abs(Math.abs(curr_yaw) - 45) > 1){
            fld.setPower(-DRIVE_SPEED / 4);
            bld.setPower(-DRIVE_SPEED / 4);
            frd.setPower(0);
            brd.setPower(0);
            telemetry.addData("Current Yaw: ", curr_yaw);
            telemetry.update();
            curr_yaw = navx_device.getYaw();
        }

        stopMotors();
        /********* Charge for the ball and park ***********/
        encoderDrive(DRIVE_SPEED,48,48,7,false);


//        encoderDrive(DRIVE_SPEED,);

//        /************ Move the servo/color sensor across the beacon ************/
//        leftBeacon.setPosition(Servo.MAX_POSITION);
//        sleep(1000);
//        int reds = 0;
//        int blues = 0;
//        for (int x = 0; x < 4; x++){
//            leftBeacon.setPosition(Servo.MAX_POSITION - (x * 0.1) );
//            if (sensorRGB.red() > sensorRGB.blue()) {
//                reds += 1;
//            }
//            if (sensorRGB.red() < sensorRGB.blue()) {
//                blues += 1;
//            }
//            sleep(500);
//        }
//
//        /************ Make the color decision ************/
//        if (reds > 2.5) {
//            telemetry.addData("Beacon is", "Red");
//            while (opModeIsActive() && range.getDistance(DistanceUnit.CM) > 1) {
//                frd.setPower(DRIVE_SPEED / 4);
//                fld.setPower(DRIVE_SPEED / 4);
//                brd.setPower(DRIVE_SPEED / 4);
//                bld.setPower(DRIVE_SPEED / 4);
//            }
//            stopMotors();
//            sleep(250);
//        }
//        else if (blues > 2.5){
//            telemetry.addData("Beacon is", "Blue");
//            leftBeacon.setPosition(Servo.MIN_POSITION + 0.1);
//
//            while (opModeIsActive() && range.getDistance(DistanceUnit.CM) > 4) {
//                frd.setPower(DRIVE_SPEED / 4);
//                fld.setPower(DRIVE_SPEED / 4);
//                brd.setPower(DRIVE_SPEED / 4);
//                bld.setPower(DRIVE_SPEED / 4);
//            }
//            stopMotors();
//            sleep(250);
//        }
//        else
//        {
//            //do nothing
//            telemetry.addData("Something is", "screwy");
//        }

    }

    public void stopMotors(){
        frd.setPower(0);
        fld.setPower(0);
        brd.setPower(0);
        bld.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean isForward) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if(!isForward){
                flipDirection(bld);
                flipDirection(brd);
                flipDirection(fld);
                flipDirection(frd);
            }

            // Determine new target position, and pass to motor controller
            newLeftTarget = bld.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = brd.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            bld.setTargetPosition(newLeftTarget);
            brd.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            bld.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            bld.setPower(speed);
            brd.setPower(speed);
            frd.setPower(speed);
            fld.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (bld.isBusy() && brd.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        bld.getCurrentPosition(),
                        brd.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            stopMotors();

            if (!isForward){
                flipDirection(bld);
                flipDirection(brd);
                flipDirection(fld);
                flipDirection(frd);
            }

            // Turn off RUN_TO_POSITION
            bld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void flipDirection(DcMotor m){
        if (m.getDirection() == DcMotor.Direction.FORWARD){
            m.setDirection(DcMotor.Direction.REVERSE);
        } else if (m.getDirection() == DcMotor.Direction.REVERSE){
            m.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void shooter(double timeoutS) throws InterruptedException {
        int shootTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            shootTarget = shoot.getCurrentPosition() + (1680);
            shoot.setTargetPosition(shootTarget);

            // Turn On RUN_TO_POSITION
            shoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            shoot.setPower(1);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (shoot.isBusy())) {
                idle();
            }

            // Stop all motion;
            shoot.setPower(0);

            // Turn off RUN_TO_POSITION
            shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

}