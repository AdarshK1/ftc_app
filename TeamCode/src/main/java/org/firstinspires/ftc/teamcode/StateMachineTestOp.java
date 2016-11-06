//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.LightSensor;
//import com.qualcomm.robotcore.hardware.UltrasonicSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.teamcode.Sensors.LightSensorObj;
//import org.firstinspires.ftc.teamcode.Sensors.SensorObj;
//
//@Autonomous(name="State Machine Test Auto", group="Robowiz")
////@Disabled
//public class StateMachineTestOp {
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    DcMotor br, bl, fr, fl;
//
//    ModernRoboticsI2cRangeSensor range;
//    LightSensor light;
//    ColorSensor color;
//    UltrasonicSensor ultra;
//
//    OpenGLMatrix lastLocation = null;
//
//    VuforiaLocalizer vuforia;
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;
//
//
//    public static void main(String[] args) {
//        SensorObj[] arr = new SensorObj[1];
////        arr[0] = new LightSensorObj();
////        HolonomicChassis holonomicChassis = new HolonomicChassis(arr);
//
//    }
//}
