package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Claw Test Tele-Op", group="TeamCode")
public class ClawTest extends OpMode{

    Servo rightTopS;
    Servo rightBottomS;
    Servo leftTopS;
    Servo leftBottomS;
    /**
     * Constructor
     */

    public ClawTest() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        rightBottomS = hardwareMap.servo.get("rbs");
        leftBottomS = hardwareMap.servo.get("lbs");
        leftTopS = hardwareMap.servo.get("lts");
        rightTopS = hardwareMap.servo.get("rts");

    }

    @Override
    public void loop() {

        if (gamepad1.x){
            rightBottomS.setPosition(0);
            rightTopS.setPosition(1);
            leftBottomS.setPosition(1);
            leftTopS.setPosition(0);
        } else if (gamepad1.a){
            rightBottomS.setPosition(1);
            rightTopS.setPosition(0);
            leftBottomS.setPosition(0);
            leftTopS.setPosition(1);
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