package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="OutreachTeleOp", group="TeamCode")
public class OutreachTeleOp extends OpMode{
DcMotor motor;
//    DcMotor frontLeftDrive;
//    DcMotor frontRightDrive;
//    DcMotor backLeftDrive;
//    DcMotor backRightDrive;
//    DcMotor spinnerRight;
//    DcMotor spinnerLeft;
//    DcMotor lift;
//    DcMotor shooter;
//    Servo tilt;
//    Servo push;


    int initialize = 0;

    int i = 0;
    double pos;

    /**
     * Constructor
     */
    public OutreachTeleOp() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
motor = hardwareMap.dcMotor.get("motor");
//        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
//        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
//        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        spinnerRight = hardwareMap.dcMotor.get("rightSpinner");
//        spinnerLeft = hardwareMap.dcMotor.get("leftSpinner");
//        lift = hardwareMap.dcMotor.get("lift");
//        tilt = hardwareMap.servo.get("tilt");
//        shooter = hardwareMap.dcMotor.get("shooter");
//        push = hardwareMap.servo.get("push");

    }

    @Override
    public void loop() {

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        float lefty = -gamepad1.left_stick_y;



        // write the values to the motors

//        if( gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
//            frontLeftDrive.setPower(gamepad1.left_stick_y);
//            backLeftDrive.setPower(gamepad1.left_stick_y);
//            frontRightDrive.setPower(gamepad1.right_stick_y);
//            backRightDrive.setPower(gamepad1.right_stick_y);
//        }
//
//        else {
//            frontRightDrive.setPower(0);
//            frontLeftDrive.setPower(0);
//            backRightDrive.setPower(0);
//            backLeftDrive.setPower(0);
//        }

        if(gamepad1.left_stick_y !=0){
            motor.setPower(gamepad1.left_stick_y);
        }
        else{
            motor.setPower(0);
        }

//        if (gamepad1.x){
//            lift.setPower(0.75);
//        }
//        else if (gamepad1.y){
//            lift.setPower(-0.75);
//        }
//        else {
//            lift.setPower(0);
//        }
//
//        if (gamepad1.a){
//            shooter.setPower(1);
//        }
//        else{
//            shooter.setPower(0);
//        }
//
//        if (gamepad1.right_bumper){
//            spinnerRight.setPower(-1);
//            spinnerLeft.setPower(1);
//        }
//
//        else if (gamepad1.left_bumper){
//            spinnerLeft.setPower(-1);
//            spinnerRight.setPower(1);
//        }
//        else {
//            spinnerLeft.setPower(0);
//            spinnerRight.setPower(0);
//        }
//
//        if (gamepad1.dpad_up){
//            if (tilt.getPosition() > 0.5) {
//                tilt.setPosition(tilt.getPosition() - 0.2);
//            }
//        }
//        else if (gamepad1.dpad_down){
//            if (tilt.getPosition() < 0.95){
//                tilt.setPosition(tilt.getPosition() + 0.2);
//            }
//        }
//
//        if (gamepad1.dpad_left){
//            push.setPosition(0.5);
//        }
//
//        else if (gamepad1.dpad_right){
//            push.setPosition(0.95);
//        }
    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    @Override
    public void stop() {
//        frontRightDrive.setPower(0);
//        frontLeftDrive.setPower(0);
//        backRightDrive.setPower(0);
//        backLeftDrive.setPower(0);
        motor.setPower(0);
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