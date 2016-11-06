//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.LightSensor;
//import com.qualcomm.robotcore.hardware.UltrasonicSensor;
//
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.teamcode.Sensors.SensorObj;
//
//import ftc.electronvolts.statemachine.StateName;
//
//public class HolonomicChassis {
//
//    DcMotorObj br, bl, fr, fl;
//    //TODO update radius properly and ticksPerRotation
//    double radius = 9.0;
//    double ticksPerRotation = 1680;
//    ModernRoboticsI2cRangeSensor range;
//    LightSensor light;
//    ColorSensor color;
//    UltrasonicSensor ultra;
//    OpenGLMatrix lastLocation = null;
//    VuforiaLocalizer vuforia;
//
//    enum states implements StateName {
//        INITIALIZING,
//        //TODO finalize states
//    };
//
//    public HolonomicChassis(SensorObj[] sensors) {
//
//        for (SensorObj sensor: sensors){
//            String sensorName = sensor.getName();
//            System.out.println(sensorName);
//        }
//
//    }
//
//    public HolonomicChassis(){}
//
//    public HolonomicChassis(DcMotorObj br, DcMotorObj bl, DcMotorObj fr, DcMotorObj fl){
//        this.br = br;
//        this.bl = bl;
//        this.fr = fr;
//        this.fl = fl;
//    }
//
//    public void rotate (double degrees) {
//        double rotations = 2 * Math.PI * radius * degrees / 360;
//        double ticks = rotations * ticksPerRotation;
//
//    }
//
//    public void setAllMotors (double power){
//        br.setPower(power);
//        fr.setPower(power);
//        bl.setPower(power);
//        fl.setPower(power);
//    }
//
//    public void stopAll() {
//        br.stop();
//        fr.stop();
//        bl.stop();
//        fl.stop();
//    }
//
//    public void move(double rightx, double righty, double leftx, double lefty) {
//        if( Math.abs(leftx) > 0 || Math.abs(lefty) > 0) {
//            fr.setPower((leftx - lefty) / 2);
//            fl.setPower((-leftx - lefty) / 2);
//            br.setPower((-leftx - lefty) / 2);
//            bl.setPower((leftx - lefty) / 2);
//        }
//
//        else if (Math.abs(rightx) > 0) {
//            fr.setPower((rightx) / 2);
//            fr.setPower((rightx) / 2);
//            br.setPower((rightx) / 2);
//            bl.setPower((rightx) / 2);
//        }
//
//        else {
//            stopAll();
//        }
//    }
//
//
//    //TODO public void move north/south
//
//    //TODO public void move east/west
//
//    //TODO public void move NE/SW
//
//    //TODO public void move NW/SE
//
//    //TODO Joystick input movement for tele-op
//
//}
