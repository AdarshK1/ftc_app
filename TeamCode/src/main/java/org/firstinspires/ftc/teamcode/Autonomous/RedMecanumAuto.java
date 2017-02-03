package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="Red Auto Mecanum", group="TeamCode")
//@Disabled
public class RedMecanumAuto extends LinearOpMode{

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
//    static final double     TURN_SPEED              = 0.5;
//    static final double     WHITE_THRESHOLD = 0.08;

//    DcMotor lift;
//    DcMotor intake;
    DcMotor shoot;
    DeviceInterfaceModule dim;

//    static final int LED_CHANNEL = 5;

//    float error;

    static final double WHITE_THRESHOLD = 0.99;

    @Override
    public void runOpMode() throws InterruptedException {

        /******** Initialize all devices ********/
        shoot = hardwareMap.dcMotor.get("shoot");

        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION);

//        chamber = hardwareMap.servo.get("chamber");
//        chamber.setPosition(Servo.MAX_POSITION);

        chamber =(CRServoImpl) hardwareMap.crservo.get("chamber");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

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

//        rangeLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");

        odsLeft = hardwareMap.opticalDistanceSensor.get("odsLeft");
        odsRight = hardwareMap.opticalDistanceSensor.get("odsRight");
        odsLeft.enableLed(true);
        odsRight.enableLed(true);

        /******** Initialize Drive *********/
        brd = hardwareMap.dcMotor.get("brd");
        bld = hardwareMap.dcMotor.get("bld");
        frd = hardwareMap.dcMotor.get("frd");
        fld = hardwareMap.dcMotor.get("fld");
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());


//        brd.setDirection(DcMotor.Direction.REVERSE);
        bld.setDirection(DcMotor.Direction.REVERSE);
//        frd.setDirection(DcMotor.Direction.REVERSE);
        fld.setDirection(DcMotor.Direction.REVERSE);

        /******* Reset Encoders ********/
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        bld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        bld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /******** Wait for Start ********/
        while (!isStarted()){
            idle();
//            telemetry.addData("Light Level", ods.getLightDetected());
//            telemetry.update();
//            odsVal.add(ods.getLightDetected());
        }



        /******** Drive forward to shoot position *******/
        encoderDrive(DRIVE_SPEED / 2, 12, 12, 10);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());
        sleep(250);
//
//        /******* Flip out cap ball thing *******/
//        flip.setPosition(Servo.MIN_POSITION);
//        sleep(5000);

        /******** Shoot two balls ********/
        shooter(10);
//        chamber.setPosition(Servo.MIN_POSITION);
        chamber.setPower(1);
        sleep(1200);
        shooter(10);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());

        /******* Strafe left one tile *******/
//        while(opModeIsActive() && (rangeLeft.getDistance(DistanceUnit.CM)) < 10){
//            //move left at .8 power
//            frd.setPower(DRIVE_SPEED);
//            fld.setPower(-DRIVE_SPEED);
//            brd.setPower(-DRIVE_SPEED);
//            bld.setPower(DRIVE_SPEED);
//            //telemetry
//            telemetry.addData("Range", rangeLeft.getDistance(DistanceUnit.CM));
//            telemetry.update();
//            idle();
//        }
//
//        //after you're 10 cm away from the side wall, stop
//        frd.setPower(0);
//        fld.setPower(0);
//        brd.setPower(0);
//        bld.setPower(0);


        frd.setPower(DRIVE_SPEED);
        fld.setPower(-DRIVE_SPEED);
        brd.setPower(-DRIVE_SPEED);
        bld.setPower(DRIVE_SPEED);

        while (range.getDistance(DistanceUnit.CM) > 25){
            telemetry.addData("Range Sensor Value; ", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        frd.setPower(0);
        fld.setPower(0);
        brd.setPower(0);
        bld.setPower(0);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());
        sleep(1000);


        /************ Make a 45 degree turn counterclockwise (towards the beacon). ************/

//        final double TOTAL_RUN_TIME_SECONDS = 5.0;
//        int DEVICE_TIMEOUT_MS = 500;
//        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
//
//        boolean finishedTurn = false;
//        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS && !finishedTurn) {
//            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
//                if ( yawPIDResult.isOnTarget() ) {
//                    telemetry.addData("Finished ", "Adjust");
//                    telemetry.update();
//                    stopMotors();
//                    finishedTurn = true;
//                } else {
//                    double output = yawPIDResult.getOutput();
//                    telemetry.addData("PID Output", output);
//                    telemetry.update();
//                    if (Math.abs(output) < 0.5){
//                        if (output < 0){
//                            output = -0.5;
//                        }
//                        else if (output > 0) {
//                            output = 0.5;
//                        }
//                    }
//                    if ( output < 0 ) {
//                        /* Rotate Left */
//                        bld.setPower(output);
//                        fld.setPower(output);
//                        brd.setPower(-output);
//                        frd.setPower(-output);
//                    } else {
//                        /* Rotate Right */
//                        bld.setPower(-output);
//                        fld.setPower(-output);
//                        brd.setPower(output);
//                        frd.setPower(output);
//                    }
//                }
//            } else {
//			          /* A timeout occurred */
//                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
//            }
//        }

        encoderDrive(DRIVE_SPEED / 2, 4, 4, 5);
        sleep(250);

        frd.setPower(DRIVE_SPEED);
        fld.setPower(-DRIVE_SPEED);
        brd.setPower(-DRIVE_SPEED);
        bld.setPower(DRIVE_SPEED);

        while (range.getDistance(DistanceUnit.CM) > 15){
            telemetry.addData("Range Sensor Value; ", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        frd.setPower(0);
        fld.setPower(0);
        brd.setPower(0);
        bld.setPower(0);
        odsVal.add(odsLeft.getLightDetected());
        odsVal.add(odsRight.getLightDetected());

        double total = 0;
        for (Double n :odsVal)
        {
            total += n;
        }
        double avg = total/(odsVal.size());

        double whiteThreshold = avg * 3;
        boolean rightHit = false;
        boolean leftHit = false;

        frd.setPower(DRIVE_SPEED / 2);
        fld.setPower(DRIVE_SPEED / 2);
        brd.setPower(DRIVE_SPEED / 2);
        bld.setPower(DRIVE_SPEED / 2);

        while ((!rightHit && !leftHit) || (!rightHit && leftHit) || (rightHit && !leftHit)){
            if (odsRight.getLightDetected() > avg){
                frd.setPower(0);
                brd.setPower(0);
                rightHit = true;
            }
            if (odsLeft.getLightDetected() > avg){
                fld.setPower(0);
                bld.setPower(0);
                leftHit = true;
            }
        }

        stopMotors();


        /******** Strafe diagonally 3 tiles until you see the white line ********/
//        frd.setDirection(DcMotor.Direction.FORWARD);
//        bld.setDirection(DcMotor.Direction.FORWARD);
//
//        frd.setPower(.8); //totally random power value?
//        bld.setPower(.8); //again, totally rand value?
//        fld.setPower(0);
//        brd.setPower(0);
//        sleep(5000);
//
//        double total = 0;
//        for (Double n :odsVal)
//        {
//            total += n;
//        }
//        double avg = total/(odsVal.size());
//
//        while (opModeIsActive() && (ods.getLightDetected() < avg * 3)){
//            telemetry.addData("Light Level",  ods.getLightDetected());
//            telemetry.update();
//            idle();
//        }
//
//        frd.setPower(0);
//        bld.setPower(0);
//        fld.setPower(0);
//        brd.setPower(0);
//
//        sleep(250);

        /******** Follow the line/use range sensor to align with beacon *******/
//        error = ods.getRawLightDetected() - WHITE_THRESHOLD;
//        turnPower = LINE_FOLLOW_KP*error;
//        leftWheel.setPower(BASE_DRIVE_POWER + turnPower);
//        rightWheel.setPower(BASE_DRIVE_POWER - turnPower);

        /******** Decide which color to push ********/

        /******** Push the correct side of the beacon ********/

        /******** Strafe back 2 tiles ********/

        /******** Use light/range sensor to ensure aligned with beacon ********/

        /******** Decide which color to push ********/

        /******** Push the correct side of the beacon ********/

        /******** Strafe backwards diagonally 3 tiles (hits cap ball) ********/

        /******** Park on center vortex ********/

    }

    public void stopMotors(){
        frd.setPower(0);
        fld.setPower(0);
        brd.setPower(0);
        bld.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

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
            bld.setPower(Math.abs(speed));
            brd.setPower(Math.abs(speed));
            frd.setPower(Math.abs(speed));
            fld.setPower(Math.abs(speed));

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
            bld.setPower(0);
            brd.setPower(0);
            frd.setPower(0);
            fld.setPower(0);

            // Turn off RUN_TO_POSITION
            bld.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
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