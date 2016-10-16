package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MechanumTest", group="TeamCode")
public class MechanumTest extends OpMode{

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor spinner;
    DcMotor lift;

    /**
     * Constructor
     */
    public MechanumTest() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        spinner = hardwareMap.dcMotor.get("spinner");
        lift = hardwareMap.dcMotor.get("lift");
    }

    @Override
    public void loop() {

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        telemetry.addData("right data", right );
        telemetry.addData("left data", left );

        // write the values to the motors

        if( Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0) {
            frontRightDrive.setPower((gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
            frontLeftDrive.setPower((-gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
            backRightDrive.setPower((-gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
            backLeftDrive.setPower((gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
        }

        else if (Math.abs(gamepad1.right_stick_x) > 0) {
            frontRightDrive.setPower((gamepad1.right_stick_x) / 2);
            frontLeftDrive.setPower((gamepad1.right_stick_x) / 2);
            backRightDrive.setPower((gamepad1.right_stick_x) / 2);
            backLeftDrive.setPower((gamepad1.right_stick_x) / 2);
        }
        else {
            frontRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            backLeftDrive.setPower(0);
        }

        if (gamepad1.a){
            spinner.setPower(1);
        }
        else if (gamepad1.b){
            spinner.setPower(-1);
        }
        else {
            spinner.setPower(0);
        }

        if (gamepad1.x){
            lift.setPower(0.75);
        }
        else  if (gamepad1.y){
            lift.setPower(-0.75);
        }
        else {
            lift.setPower(0);
        }
    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    @Override
    public void stop() {
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 0.85, 0.85 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}