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
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navx.ftc.*;
import org.firstinspires.ftc.teamcode.navx.*;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

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

@Autonomous(name="Red Auto", group="Robowiz")
@Disabled
public class RedAuto extends LinearOpMode {

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
    OpticalDistanceSensor ods;
    TouchSensor touch;

    DcMotor br, bl, fr, fl;

    ModernRoboticsI2cRangeSensor range;
    LightSensor light;
    ColorSensor color;
    UltrasonicSensor ultra;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    private final int NAVX_DIM_I2C_PORT = 1;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private final double TARGET_ANGLE_DEGREES = -45.0;
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
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;
    static final double     WHITE_THRESHOLD = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        spinnerRight = hardwareMap.dcMotor.get("rightSpinner");
        spinnerLeft = hardwareMap.dcMotor.get("leftSpinner");
        lift = hardwareMap.dcMotor.get("lift");
        tilt = hardwareMap.servo.get("tilt");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        push = hardwareMap.servo.get("push");
        push.setPosition(0);
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        touch = hardwareMap.touchSensor.get("touch");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);



        /* If possible, use encoders when driving, as it results in more */
        /* predicatable drive system response.                           */
//        leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        navx_device.zeroYaw();

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        idle();
//
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Path0", "Starting at %7d :%7d",
//                backLeftDrive.getCurrentPosition(),
//                backRightDrive.getCurrentPosition());
//        telemetry.update();

//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AV/at7D/////AAAAGeh38frwBEGUke6F4wbsl2eI/QsNUvPFSZZGsy3+2Tifa5qSfnCh93gmS0KSfS526VeacacNr5M3kk64htoLkR0k4nIyccwt4vAvID76fniTyv5ykj7AFVdPdm3HRf8dn4Kd/MYmsVCnoKeklJvUlPkRBf6W1vBa63dF75Fc8H15e9+s5q3PHaz/jrrdzVaXm4yZB0f/vBmsA1kw8ERWrPhD1ZYP4T2mpzpRAQvxNTBBc9yNzSQ8kbEm6a0SN8qviw8EQofAzrtL5iwlF8V0e21Ldjn5SCh9qRcn0LBz6olZYHU+yjPB6qlabBFpM76eEUgUMeE8CiyTVK0SkB006QJKSHyvMQQd7+ds+LMztKoe";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", ods.getLightDetected());
            telemetry.update();
            idle();
        }
        encoderDrive(DRIVE_SPEED, 12, 12, 10);

        sleep(250);

        final double TOTAL_RUN_TIME_SECONDS = 10.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS ) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    setDrivePower(0);
                } else {
                    double output = yawPIDResult.getOutput();
                    telemetry.addData("PID Output", output);
                    telemetry.update();
                    if (Math.abs(output) < 0.2){
                        if (output < 0){
                            output = -0.2;
                        }
                        else if (output > 0) {
                            output = 0.2;
                        }
                    }
                    if ( output < 0 ) {
                        /* Rotate Left */
                        backLeftDrive.setPower(-output);
                        frontLeftDrive.setPower(-output);
                        backRightDrive.setPower(output);
                        frontRightDrive.setPower(output);
                    } else {
                        /* Rotate Right */
                        backLeftDrive.setPower(output);
                        frontLeftDrive.setPower(output);
                        backRightDrive.setPower(-output);
                        frontRightDrive.setPower(-output);
                    }
                }
            } else {
			          /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }

        encoderDrive(DRIVE_SPEED, 36, 36, 10);

        setDrivePower(DRIVE_SPEED);

        while (opModeIsActive() && (ods.getLightDetected() < WHITE_THRESHOLD)){
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  ods.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        setDrivePower(0);

        sleep(250);

        yawPIDController.setSetpoint(-90.0);
        yawPIDResult = new navXPIDController.PIDResult();

        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS ) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    setDrivePower(0);
                } else {
                    double output = yawPIDResult.getOutput();
                    telemetry.addData("PID Output", output);
                    telemetry.update();
                    if (Math.abs(output) < 0.2){
                        if (output < 0){
                            output = -0.2;
                        }
                        else if (output > 0) {
                            output = 0.2;
                        }
                    }
                    if ( output < 0 ) {
                        /* Rotate Left */
                        backLeftDrive.setPower(-output);
                        frontLeftDrive.setPower(-output);
                        backRightDrive.setPower(output);
                        frontRightDrive.setPower(output);
                    } else {
                        /* Rotate Right */
                        backLeftDrive.setPower(output);
                        frontLeftDrive.setPower(output);
                        backRightDrive.setPower(-output);
                        frontRightDrive.setPower(-output);
                    }
                }
            } else {
			          /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }

        sleep(250);
        encoderDrive(-DRIVE_SPEED, 24, 24, 15.0);

//        while (range.getDistance(DistanceUnit.CM) > 10 ){
//            setDrivePower(DRIVE_SPEED / 2);
//        }

//        setDrivePower(0);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);  // S1: Forward 48 Inches with 5 Sec timeout
//        sleep(2000);
//        encoderDrive(DRIVE_SPEED, 24, 24, 5.0);

//        setDrivePower(DRIVE_SPEED);
//
//        while (opModeIsActive() && (ods.getLightDetected() < WHITE_THRESHOLD)){
//            // Display the light level while we are looking for the line
//            telemetry.addData("Light Level",  ods.getLightDetected());
//            telemetry.update();
//            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
//        }
//
//        setDrivePower(0);
//
//        while (opModeIsActive() && !touch.isPressed() && range.getDistance(DistanceUnit.CM) > 5){
//            if (ods.getLightDetected() < WHITE_THRESHOLD){
//                backRightDrive.setPower(DRIVE_SPEED);
//                frontRightDrive.setPower(DRIVE_SPEED);
//                backLeftDrive.setPower(0);
//                frontLeftDrive.setPower(0);
//            }
//            else if (ods.getLightDetected() > WHITE_THRESHOLD){
//                backRightDrive.setPower(0);
//                frontRightDrive.setPower(0);
//                backLeftDrive.setPower(DRIVE_SPEED);
//                frontLeftDrive.setPower(DRIVE_SPEED);
//            }
//            else {
//                setDrivePower(0);
//            }
//
//        }

//        setDrivePower(0);
//        encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//        sleep(1000);     // pause for servos to move

//        telemetry.addData("Path", "Complete");
//        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void setDrivePower(double p){
        backLeftDrive.setPower(p);
        backRightDrive.setPower(p);
        frontLeftDrive.setPower(p);
        frontRightDrive.setPower(p);
    }


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
