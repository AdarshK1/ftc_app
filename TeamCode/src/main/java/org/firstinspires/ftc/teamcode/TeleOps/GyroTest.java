package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;

/**
 * Created by akulkarni on 2/3/17.
 */
@Disabled
@TeleOp(name="Gyro Test Tele-Op", group="TeamCode")
public class GyroTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor fld;
    DcMotor frd;
    DcMotor bld;
    DcMotor brd;

    private final int NAVX_DIM_I2C_PORT = 5;
    private AHRS navx_device;

    double r;
    double robotAngle;
    double rightX;

    double v1;
    double v2;
    double v3;
    double v4;

    double gyroVal;
    double thresholdVal;
    double targetAngle;
    double output;

    public GyroTest() {

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

        thresholdVal = 0.01;

//        Mechanum Drive


        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        navx_device.zeroYaw();


    }

    @Override
    public void loop(){

        r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;

        v1 = r * Math.cos(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.sin(robotAngle) + rightX;
        v4 = r * Math.cos(robotAngle) - rightX;

        fld.setPower(v1);
        frd.setPower(v2);
        bld.setPower(v3);
        brd.setPower(v4);

        telemetry.addData("FLD Power: ", fld.getPower());
        telemetry.addData("FRD Power: ", frd.getPower());
        telemetry.addData("BLD Power: ", bld.getPower());
        telemetry.addData("BRD Power: ", brd.getPower());



        telemetry.addData("Gyro Raw Value: ", navx_device.getYaw());
        telemetry.update();

        if (gamepad1.b) {
            navx_device.zeroYaw();
        }
        else if(gamepad1.a){
            gyroVal = navx_device.getYaw();
            output = 0.1;
            while (Math.abs(gyroVal-targetAngle) > thresholdVal){
                gyroVal = navx_device.getYaw();

                if(gyroVal-targetAngle > 0) {
                    bld.setPower(output);
                    fld.setPower(output);
                    brd.setPower(-output);
                    frd.setPower(-output);
                }
                else if(gyroVal-targetAngle < 0) {
                    bld.setPower(-output);
                    fld.setPower(-output);
                    brd.setPower(output);
                    frd.setPower(output);

                }

            }
        }
    }


}
