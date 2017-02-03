/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.util.ArrayList;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue2 Auto", group="Robowiz")
@Disabled
public class Blue2Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor spinnerRight;
    DcMotor spinnerLeft;
    DcMotor lift;
    DcMotor shooter;
    Servo tilt;
    Servo push;
    Servo flip;
    Servo button;
    ColorSensor sensorRGB;
    ModernRoboticsI2cRangeSensor range;
    OpticalDistanceSensor ods;
    TouchSensor touch;
    DeviceInterfaceModule dim;

    ArrayList<Double> odsVal = new ArrayList<Double>();
    static final int LED_CHANNEL = 5;
    private final int NAVX_DIM_I2C_PORT = 1;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private final double TARGET_ANGLE_DEGREES = 45.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     WHITE_THRESHOLD = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {

        /*********** Initialize all devices, sensors, IMU and FTCVision. ************/

        /** Initialize Drive **/
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        /** Initialize Lift and Spinner **/
        spinnerRight = hardwareMap.dcMotor.get("rightSpinner");
        spinnerLeft = hardwareMap.dcMotor.get("leftSpinner");
        lift = hardwareMap.dcMotor.get("lift");

        /** Servos and Shooter **/
        tilt = hardwareMap.servo.get("tilt");
        tilt.setPosition(Servo.MAX_POSITION);
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        push = hardwareMap.servo.get("push");
        push.setPosition(Servo.MIN_POSITION + 0.1);
        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION - 0.2);

        button = hardwareMap.servo.get("button");
        button.setPosition(Servo.MIN_POSITION);

        /** Initialize Sensors and IMU **/
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        touch = hardwareMap.touchSensor.get("touch");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        navx_device.zeroYaw();

        boolean bLedOn = true;

        dim = hardwareMap.deviceInterfaceModule.get("dim");
        dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("color");
        dim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        /************ Create a PID Controller which uses the Yaw Angle as input ************/
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /************ Wait for Start. ************/
        while (!isStarted()) {
            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", ods.getLightDetected());
            telemetry.update();
            odsVal.add(ods.getLightDetected());
            idle();
        }

        /************ Drive Forward for 12 inches. ************/
        encoderDrive(DRIVE_SPEED, 12, 12, 10);
        odsVal.add(ods.getLightDetected());
        sleep(250);

        /************ Make a 45 degree turn counterclockwise (towards the beacon). ************/
        final double TOTAL_RUN_TIME_SECONDS = 10.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
//
        boolean finishedTurn = false;
        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS && !finishedTurn) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    setDrivePower(0);
                    finishedTurn = true;
                } else {
                    double output = yawPIDResult.getOutput();
                    telemetry.addData("PID Output", output);
                    telemetry.update();
                    if (Math.abs(output) < 0.5){
                        if (output < 0){
                            output = -0.5;
                        }
                        else if (output > 0) {
                            output = 0.5;
                        }
                    }
                    if ( output < 0 ) {
                        /* Rotate Left */
                        backLeftDrive.setPower(output);
                        frontLeftDrive.setPower(output);
                        backRightDrive.setPower(-output);
                        frontRightDrive.setPower(-output);
                    } else {
                        /* Rotate Right */
                        backLeftDrive.setPower(-output);
                        frontLeftDrive.setPower(-output);
                        backRightDrive.setPower(output);
                        frontRightDrive.setPower(output);
                    }
                }
            } else {
			          /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }

        navx_device.close();
        odsVal.add(ods.getLightDetected());
        sleep(250);


        /************ Forward 26 inches. ************/
        encoderDrive(DRIVE_SPEED, 26, 26, 10);
        odsVal.add(ods.getLightDetected());
        tilt.setPosition(Servo.MAX_POSITION - 0.03);
        sleep(250);

/************  Flip out cap ball holder so ball doesn't hit it. ************/
        flip.setPosition(Servo.MIN_POSITION);
        odsVal.add(ods.getLightDetected());
        sleep(500);

        /************ Shoot! ************/
        shooter.setPower(1);
        sleep(1500);
        shooter.setPower(0);
        odsVal.add(ods.getLightDetected());
        sleep(250);

        /************* Forward 10 inches again to get in vicinity of white line/beacon. ************/
        encoderDrive(DRIVE_SPEED, 10, 10, 10);
        odsVal.add(ods.getLightDetected());
        sleep(250);


        /************ Forward till the white line ************/
        setDrivePower(DRIVE_SPEED);
        double total = 0;
        for (Double n :odsVal)
        {
            total += n;
        }
        double avg = total/(odsVal.size());


        while (opModeIsActive() && (ods.getLightDetected() < avg * 3)){
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  ods.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        setDrivePower(0);
        sleep(250);

        /************* A little bit of forward offset to help line up the camera post-turn. ************/
//        encoderDrive(DRIVE_SPEED, 2, 2, 5);

        /************ 45 degree turn to line up camera with beacon. ************/
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        navx_device.zeroYaw();

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES - 10);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        yawPIDResult = new navXPIDController.PIDResult();
        finishedTurn = false;
        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS && !finishedTurn) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    setDrivePower(0);
                    finishedTurn = true;
                } else {
                    double output = yawPIDResult.getOutput();
                    telemetry.addData("PID Output", output);
                    telemetry.update();
                    if (Math.abs(output) < 0.5){
                        if (output < 0){
                            output = -0.5;
                        }
                        else if (output > 0) {
                            output = 0.5;
                        }
                    }
                    if ( output < 0 ) {
                        /* Rotate Left */
//                        backLeftDrive.setPower(-output);
//                        frontLeftDrive.setPower(-output);
                        backRightDrive.setPower(output);
                        frontRightDrive.setPower(output);
                    } else {
                        /* Rotate Right */
//                        backLeftDrive.setPower(output);
//                        frontLeftDrive.setPower(output);
                        backRightDrive.setPower(-output);
                        frontRightDrive.setPower(-output);
                    }
                }
            } else {
			          /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }

        navx_device.close();


        /************ Set the robot 8 cm from the wall ************/
        if (range.getDistance(DistanceUnit.CM) > 8){
            setDrivePower(DRIVE_SPEED /2);
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        else {
            setDrivePower(-DRIVE_SPEED /2);
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

//        commented out by Radhika to be replaced with timed drive
        while (opModeIsActive() && Math.abs(range.getDistance(DistanceUnit.CM) - 8) > 1){
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle();
        }


//        setDrivePower(-DRIVE_SPEED / 2);
//        sleep(500);
        setDrivePower(0);


        /************ Move the servo/color sensor across the beacon ************/
        button.setPosition(Servo.MAX_POSITION);
        sleep(1000);
        int reds = 0;
        int blues = 0;
        for (int x = 0; x < 4; x++){
            button.setPosition(Servo.MAX_POSITION - (x * 0.1) );
            if (sensorRGB.red() > sensorRGB.blue()) {
                reds += 1;
            }
            if (sensorRGB.red() < sensorRGB.blue()) {
                blues += 1;
            }
            sleep(500);
        }


        /************ Make the color decision ************/
        if (reds > 2.5) {
            telemetry.addData("Beacon is", "Red");
            button.setPosition(Servo.MIN_POSITION + 0.1);
            while (opModeIsActive() && range.getDistance(DistanceUnit.CM) > 1) {
                setDrivePower(DRIVE_SPEED / 2);
            }
            setDrivePower(0);
            sleep(250);
        }
        else if (blues > 2.5){
            telemetry.addData("Beacon is", "Blue");
            //TODO Check this position


            while (opModeIsActive() && range.getDistance(DistanceUnit.CM) > 1) {
                setDrivePower(DRIVE_SPEED / 2);
            }
            setDrivePower(0);
            sleep(250);
        }
        else
        {
            //do nothing
            telemetry.addData("Something is", "screwy");
        }

        while(opModeIsActive()) {
            telemetry.update();
        }

        button.setPosition(Servo.MAX_POSITION);
        sleep(1000);
        int beaconLeft = 0;
        // beaconLeft = 1 means red
        // beaconLeft = 0 means blue
        if (sensorRGB.red() > sensorRGB.blue()) {
            beaconLeft = 1;
            sleep(500);
        }

        if (beaconLeft == 0) {
            button.setPosition(Servo.MIN_POSITION);
            sleep(1000);
        }

        setDrivePower(DRIVE_SPEED / 2);
        sleep(1000);
        setDrivePower(0);

        while (opModeIsActive()){
            telemetry.addData("position: ", tilt.getPosition());
            telemetry.addData("Raw", ods.getRawLightDetected());
            telemetry.addData("Button", button.getPosition());
            telemetry.addData("Normal", ods.getLightDetected());
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.update();
        }

        /** Charge backwards to hit the cap ball **/
//        setDrivePower(-.9);
//        sleep(3000);
//        setDrivePower(0);
//
//        setDrivePower(0.9);
//        sleep(500);
//        setDrivePower(0);
//
//        sleep(1000);
//
//        setDrivePower(-0.9);
//        sleep(1000);
//        setDrivePower(0);
//
//        backLeftDrive.setPower(-0.9);
//        frontLeftDrive.setPower(-0.9);
//        sleep(3000);
//        setDrivePower(0);

 }

    public void setDrivePower(double p){
        backLeftDrive.setPower(p);
        backRightDrive.setPower(p);
        frontLeftDrive.setPower(p);
        frontRightDrive.setPower(p);
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backLeftDrive.setTargetPosition(newLeftTarget);
            backRightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            frontLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            backLeftDrive.getCurrentPosition(),
                                            backRightDrive.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            frontLeftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              sleep(250);   // optional pause after each move
        }
    }
}
