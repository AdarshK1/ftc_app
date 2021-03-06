package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name="Mechanum Tele-Op", group="TeamCode")
public class MechanumTeleOp extends OpMode{

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor fld;
    DcMotor frd;
    DcMotor bld;
    DcMotor brd;

    DcMotor lift;
    DcMotor intake;
    DcMotor shoot;
    DcMotor dcSpinner;

    ModernRoboticsI2cRangeSensor leftRange;
    ModernRoboticsI2cRangeSensor rightRange;

    DeviceInterfaceModule dim;
    ColorSensor color;

    Servo flip;
    Servo release;
    Servo leftBeacon;
//    Servo rightBeacon;
    CRServoImpl chamber;
    CRServoImpl servoSpinner;

    static final int LED_CHANNEL = 5;

    int state = 0;
    long time = System.currentTimeMillis();
    boolean machine = true;
    boolean limitHit = false;

    boolean intakeOn;
//
    /**
     * Constructor
     */
    public MechanumTeleOp() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        brd = hardwareMap.dcMotor.get("brd");
//        brd.setDirection(DcMotor.Direction.REVERSE);
        brd.setDirection(DcMotor.Direction.REVERSE);
        bld = hardwareMap.dcMotor.get("bld");
        bld.setDirection(DcMotor.Direction.FORWARD);

        frd = hardwareMap.dcMotor.get("frd");
        frd.setDirection(DcMotor.Direction.REVERSE);
        fld = hardwareMap.dcMotor.get("fld");
        fld.setDirection(DcMotor.Direction.FORWARD);

        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION - 0.2);
        release = hardwareMap.servo.get("release");
        release.setPosition(Servo.MIN_POSITION + 0.1);

        color = hardwareMap.colorSensor.get("color");

        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");

        leftBeacon = hardwareMap.servo.get("leftBeacon");
        leftBeacon.setPosition(Servo.MAX_POSITION);
//        rightBeacon = hardwareMap.servo.get("rightBeacon");
//        rightBeacon.setPosition(Servo.MIN_POSITION);

        dcSpinner = hardwareMap.dcMotor.get("dcSpinner");

        servoSpinner = (CRServoImpl) hardwareMap.crservo.get("servoSpinner");

        chamber =(CRServoImpl) hardwareMap.crservo.get("chamber");

        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        shoot = hardwareMap.dcMotor.get("shoot");
        intakeOn = false;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
    }

    @Override
    public void loop() {

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        if (gamepad1.start && !machine && (System.currentTimeMillis() - time) > 500){
            state = 1;
            telemetry.addData("State: ", state);
            telemetry.update();
            machine = true;
            time = System.currentTimeMillis();
        }

        else if (gamepad1.start && machine && (System.currentTimeMillis() - time) > 500){
            state = 0;
            telemetry.addData("State: ", state);
            telemetry.update();
            machine = false;
            time = System.currentTimeMillis();
        }

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        fld.setPower(v1);
        frd.setPower(v2);
        bld.setPower(v3);
        brd.setPower(v4);

        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Green: ", color.green());
        telemetry.addData("Alpha: ", color.alpha());
        telemetry.addData("ARGB: ", color.argb());

        telemetry.addData("Left Range: ", leftRange.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Range: ", rightRange.getDistance(DistanceUnit.CM));

        telemetry.addData("FLD Power: ", v1);
        telemetry.addData("FRD Power: ", v2);
        telemetry.addData("BLD Power: ", v3);
        telemetry.addData("BRD Power: ", v4);

        telemetry.addData("rightTrigger", gamepad1.right_trigger);
        telemetry.addData("leftTrigger", gamepad1.left_trigger);

        telemetry.update();

//         write the values to the motors

//        if( Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0) {
//            frd.setPower((gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
//            fld.setPower((-gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
//            brd.setPower((-gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
//            bld.setPower((gamepad1.left_stick_x - gamepad1.left_stick_y) / 2);
//        }
//
//        else if (Math.abs(gamepad1.right_stick_x) > 0) {
//            frd.setPower((gamepad1.right_stick_x) / 2);
//            fld.setPower(-(gamepad1.right_stick_x) / 2);
//            brd.setPower((gamepad1.right_stick_x) / 2);
//            bld.setPower(-(gamepad1.right_stick_x) / 2);
//        }
//        else {
//            frd.setPower(0);
//            fld.setPower(0);
//            brd.setPower(0);
//            bld.setPower(0);
//        }


        if (state %2 == 0) {
            dim.setLED(1, true);
            dim.setLED(0, false);

            telemetry.addLine("Red Mode");
            telemetry.update();
//            if (gamepad1.b && !intakeOn) {
//                long time = System.currentTimeMillis();
//                for (int i = 0; i < 4; i++) {
//                    while ((System.currentTimeMillis() - time) < 200) {
//                        int x = 0;
//                    }
//                    intake.setPower(intake.getPower() + 0.2);
//                }
//                intakeOn = true;
//            }  else if (gamepad1.b && intakeOn) {
//                long time = System.currentTimeMillis();
//                for (int i = 0; i < 4; i++) {
//                    while ((System.currentTimeMillis() - time) < 200) {
//                        int x = 0;
//                    }
//                    intake.setPower(intake.getPower() - 0.2);
//                }
//                intakeOn = false;
//                intake.setPower(0);
//            }  else if (gamepad1.a && intakeOn) {
//                long time = System.currentTimeMillis();
//                for (int i = 0; i < 4; i++) {
//                    while ((System.currentTimeMillis() - time) < 200) {
//                        int x = 0;
//                    }
//                    intake.setPower(intake.getPower() + 0.2);
//                }
//                intakeOn = false;
//                intake.setPower(0);
//            }

            if (gamepad1.right_trigger > 0.9 && !intakeOn) {
                long time = System.currentTimeMillis();
                for (int i = 0; i < 4; i++) {
                    while ((System.currentTimeMillis() - time) < 200) {
                        int x = 0;
                    }
                    intake.setPower(intake.getPower() + 0.2);
                }
                intakeOn = true;
            }
            else if(gamepad1.right_bumper && !intakeOn){
                long time = System.currentTimeMillis();
                for (int i = 0; i < 4; i++) {
                    while ((System.currentTimeMillis() - time) < 200) {
                        int x = 0;
                    }
                    intake.setPower(intake.getPower() - 0.2);
                }
                intakeOn = true;
            }
            else if (gamepad1.right_trigger < 0.1 && intakeOn && !gamepad1.right_bumper) {
                long time = System.currentTimeMillis();
                for (int i = 0; i < 4; i++) {
                    while ((System.currentTimeMillis() - time) < 200) {
                        int x = 0;
                    }
                    intake.setPower(intake.getPower() - 0.2);
                }
                intakeOn = false;
                intake.setPower(0);
            }

            if (gamepad1.a) {
                leftBeacon.setPosition(Servo.MAX_POSITION);
            } else if (gamepad1.b) {
                leftBeacon.setPosition(Servo.MIN_POSITION + 0.1);
            }

            if (gamepad1.x) {
                shoot.setPower(1);
            }
            else{
                shoot.setPower(0);
            }

            if (gamepad1.dpad_up) {
                flip.setPosition(Servo.MIN_POSITION + 0.5);
            }


            if (gamepad1.dpad_down) {
                flip.setPosition(Servo.MAX_POSITION);
            }

            if (gamepad1.dpad_right) {
                chamber.setPower(-1);
            }
            else if (gamepad1.dpad_left) {
                chamber.setPower(1);
            }
            else
            {
                chamber.setPower(0);
            }
        }

        else if (state %2 == 1) {
            dim.setLED(1, false);
            dim.setLED(0, true);
            telemetry.addLine("Blue Mode");
            telemetry.update();

            if (gamepad1.x) {
                lift.setPower(1.0);
            } else if (gamepad1.y) {
                lift.setPower(-1.0);
            }

            if (gamepad1.left_bumper) {
                dcSpinner.setPower(1);
                servoSpinner.setPower(-1);
            } else if (gamepad1.right_bumper) {
                dcSpinner.setPower(-1);
                servoSpinner.setPower(1);
            } else {
                lift.setPower(0);
                servoSpinner.setPower(0);
                dcSpinner.setPower(0);
            }

            if (gamepad1.a){
                release.setPosition(Servo.MAX_POSITION);
            } else if (gamepad1.b){
                release.setPosition(Servo.MIN_POSITION);
            }
        }
    }


	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    @Override
    public void stop() {
        frd.setPower(0);
        fld.setPower(0);
        brd.setPower(0);
        bld.setPower(0);
        flip.close();
        chamber.close();
        servoSpinner.close();
        release.close();
        leftBeacon.close();
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

    public void shooter(double timeoutS) {
        int shootTarget;

        // Determine new target position, and pass to motor controller
        shootTarget = shoot.getCurrentPosition() + (1680);
        shoot.setTargetPosition(shootTarget);

        // Turn On RUN_TO_POSITION
        shoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        shoot.setPower(1);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while ((runtime.seconds() < timeoutS) && (shoot.isBusy())) {
            Thread.yield();
        }

        // Stop all motion;
        shoot.setPower(0);

        // Reset to loading position;
        shootTarget = shoot.getCurrentPosition() + (1260);
        shoot.setTargetPosition(shootTarget);

        // Turn off RUN_TO_POSITION
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}