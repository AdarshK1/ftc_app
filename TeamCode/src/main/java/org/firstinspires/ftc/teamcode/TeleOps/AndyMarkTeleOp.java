package org.firstinspires.ftc.teamcode.TeleOps;


import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name="AndymarkTexleOp", group="TeamCode")
public class AndyMarkTeleOp extends OpMode{

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor spinnerRight;
    DcMotor spinnerLeft;
    DcMotor lift;
    DcMotor shooter;
    Servo tilt;
    Servo flip;
    Servo push;
    Servo button;
    OpticalDistanceSensor ods;
    ModernRoboticsI2cRangeSensor range;
    DeviceInterfaceModule dim;
    AnalogInput limit;


    ColorSensor sensorRGB;


    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;

    int state = 0;
    long time = System.currentTimeMillis();
    boolean machine = true;
    boolean limitHit = false;

    /**
     * Constructor
     */
    public AndyMarkTeleOp() {

}

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {



        backRightDrive = hardwareMap.dcMotor.get("brd");
        backLeftDrive = hardwareMap.dcMotor.get("bld");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive = hardwareMap.dcMotor.get("frd");
        frontLeftDrive = hardwareMap.dcMotor.get("fld");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        spinnerRight = hardwareMap.dcMotor.get("rightSpinner");
        spinnerLeft = hardwareMap.dcMotor.get("leftSpinner");
        lift = hardwareMap.dcMotor.get("lift");
        shooter = hardwareMap.dcMotor.get("shooter"); shooter.setDirection(DcMotor.Direction.REVERSE);

        tilt = hardwareMap.servo.get("tilt");
        tilt.setPosition(Servo.MAX_POSITION);
        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION - 0.2);
        button = hardwareMap.servo.get("button");
        button.setPosition(Servo.MIN_POSITION);

        push = hardwareMap.servo.get("push");
        push.setPosition(Servo.MIN_POSITION);

        ods = hardwareMap.opticalDistanceSensor.get("ods");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        dim = hardwareMap.deviceInterfaceModule.get("dim");

        dim = hardwareMap.deviceInterfaceModule.get("dim");
        limit = new AnalogInput(dim, 7);

        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our DeviceInterfaceModule object.
        dim = hardwareMap.deviceInterfaceModule.get("dim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("color");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        dim.setDigitalChannelState(LED_CHANNEL, bLedOn);

    }

    @Override
    public void loop() {

        dim.setDigitalChannelState(LED_CHANNEL, false);

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        if (limit.getVoltage() > 4) {
            limitHit = true;
        }
        else if (limit.getVoltage() == 0){
            limitHit = false;
        }

        float lefty = -gamepad1.left_stick_y;
        float righty = -gamepad1.right_stick_y;
        if (gamepad1.start && machine == false && (System.currentTimeMillis() - time) > 500){
                state = 1;
                telemetry.addData("State: ", state);
            telemetry.update();
                machine = true;
                time = System.currentTimeMillis();
        }

        else if (gamepad1.start == true && machine == true && (System.currentTimeMillis() - time) > 500){
            state = 0;
            telemetry.addData("State: ", state);
            telemetry.update();
            machine = false;
            time = System.currentTimeMillis();
        }


        /***  state = 0 is red mode, state = 1 is blue mode  ***/

        if (state %2 == 0) {
            dim.setLED(1, true);
            dim.setLED(0, false);
            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                frontLeftDrive.setPower(gamepad1.left_stick_y);
                backLeftDrive.setPower(gamepad1.left_stick_y);
                frontRightDrive.setPower(gamepad1.right_stick_y);
                backRightDrive.setPower(gamepad1.right_stick_y);
            } else {
                frontRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                backLeftDrive.setPower(0);
            }

            if (gamepad1.a) {
                shooter.setPower(1);
            } else if (gamepad1.b) {
                shooter.setPower(-1);
            } else {
                shooter.setPower(0);
            }

            if (gamepad1.x){
                if (button.getPosition() > 0.2) {
                    button.setPosition(Servo.MIN_POSITION + .2);
//                    button position code -> might need to be max
                }
            }

            else if (gamepad1.y) {
                if (button.getPosition() < 0.98) {
                    button.setPosition(Servo.MAX_POSITION);
//                    button position code -> might need to be min
                }
            }

            if (gamepad1.dpad_up) {
                if (tilt.getPosition() > 0.2) {
                    tilt.setPosition(tilt.getPosition() - 0.01);
                }
            } else if (gamepad1.dpad_down) {
                if (tilt.getPosition() < 0.98) {
                    tilt.setPosition(tilt.getPosition() + 0.01);
                }
            }

            if (gamepad1.dpad_left) {
                if (push.getPosition() < 0.98) {
                    push.setPosition(push.getPosition() + 0.01);
                }
            } else if (gamepad1.dpad_right) {
                if (push.getPosition() > 0.02) {
                    push.setPosition(push.getPosition() - 0.01);
                }
            }

            if (gamepad1.left_bumper){
                flip.setPosition(Servo.MIN_POSITION );
//                flip.setPosition(Servo.MAX_POSITION);
            }

            if (gamepad1.right_bumper){
                flip.setPosition(Servo.MAX_POSITION -0.2);
            }
        }
        /*** state = 1 is blue mode (0 is red mode)  ***/
        else if (state %2 == 1){
            dim.setLED(1, false);
            dim.setLED(0, true);

            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                frontLeftDrive.setPower(gamepad1.left_stick_y);
                backLeftDrive.setPower(gamepad1.left_stick_y);
                frontRightDrive.setPower(gamepad1.right_stick_y);
                backRightDrive.setPower(gamepad1.right_stick_y);
            }
            else if (gamepad1.dpad_left ){
                frontLeftDrive.setPower(0.3);
                backLeftDrive.setPower(0.3);

                frontRightDrive.setPower(-0.3);
                backRightDrive.setPower(-0.3);
            }
            else if (gamepad1.dpad_right){
                frontLeftDrive.setPower(-0.3);
                backLeftDrive.setPower(-0.3);

                frontRightDrive.setPower(0.3);
                backRightDrive.setPower(0.3);
            }
            else {
                frontRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                backLeftDrive.setPower(0);
            }

            if (gamepad1.x) {
                lift.setPower(0.75);
            } else if (gamepad1.y && !limitHit) {
                lift.setPower(-0.75);
            } else {
                lift.setPower(0);
            }

            if (gamepad1.right_bumper) {
                spinnerRight.setPower(-1);
                spinnerLeft.setPower(1);
            } else if (gamepad1.left_bumper) {
                spinnerLeft.setPower(-1);
                spinnerRight.setPower(1);
            } else {
                spinnerLeft.setPower(0);
                spinnerRight.setPower(0);
            }





        }

        telemetry.addData("position: ", tilt.getPosition());
        telemetry.addData("Raw", ods.getRawLightDetected());
        telemetry.addData("Button", button.getPosition());
        telemetry.addData("Normal", ods.getLightDetected());
        telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.addData("LTrigger", gamepad1.left_trigger);
        telemetry.update();
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
        lift.setPower(0);
        shooter.setPower(0);
        spinnerLeft.setPower(0);
        spinnerRight.setPower(0);
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