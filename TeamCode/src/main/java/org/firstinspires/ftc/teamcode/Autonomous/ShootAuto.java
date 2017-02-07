package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.util.ArrayList;

/**
 * Created by akulkarni on 2/5/17.
 */

@Autonomous(name="Shoot Auto", group="TeamCode")
//@Disabled
public class ShootAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor fld;
    DcMotor frd;
    DcMotor bld;
    DcMotor brd;

    Servo flip;

    CRServoImpl chamber;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;

    DcMotor shoot;

    @Override
    public void runOpMode() throws InterruptedException {

        /******** Initialize all devices ********/
        shoot = hardwareMap.dcMotor.get("shoot");

        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION);

        chamber =(CRServoImpl) hardwareMap.crservo.get("chamber");

        /******** Initialize Drive *********/
        brd = hardwareMap.dcMotor.get("brd");
        bld = hardwareMap.dcMotor.get("bld");
        frd = hardwareMap.dcMotor.get("frd");
        fld = hardwareMap.dcMotor.get("fld");

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

        /******** Wait for Start ********/
        while (!isStarted()){
            idle();
        }

        /******** Drive forward to shoot position *******/
        encoderDrive(DRIVE_SPEED / 2, 12, 12, 10);
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

        stopMotors();

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