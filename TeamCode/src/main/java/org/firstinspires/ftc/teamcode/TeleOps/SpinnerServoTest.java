package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Spinner Test Tele-Op", group="TeamCode")
public class SpinnerServoTest extends OpMode{

//    CRServoImpl rightS;
//    CRServoImpl leftS;
    CRServoImpl servoSpinner;
    DcMotor dcSpinner;
    /**
     * Constructor
     */
    public SpinnerServoTest() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
//        rightS = (CRServoImpl) hardwareMap.crservo.get("rs");
//        leftS = (CRServoImpl) hardwareMap.crservo.get("ls");
        servoSpinner = (CRServoImpl) hardwareMap.crservo.get("servoSpinner");
        dcSpinner = hardwareMap.dcMotor.get("dcSpinner");



    }

    @Override
    public void loop() {

        if (gamepad1.x){
            servoSpinner.setPower(1);
            dcSpinner.setPower(-1);
        } else if (gamepad1.a){
            servoSpinner.setPower(1);
            dcSpinner.setPower(-1);
        } else {
            servoSpinner    .setPower(0);
            dcSpinner.setPower(0);
        }

    }


	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */

    @Override
    public void stop() {

    }

}