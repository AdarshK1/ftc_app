package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.ArrayList;

/**
 * Linear Vision Sample
 * <p/>
 * Use this in a typical linear op mode. A LinearVisionOpMode allows using
 * Vision Extensions, which do a lot of processing for you. Just enable the extension
 * and set its options to your preference!
 * <p/>
 * Please note that the LinearVisionOpMode is specially designed to target a particular
 * version of the FTC Robot Controller app. Changes to the app may break the LinearVisionOpMode.
 * Should this happen, open up an issue on GitHub. :)
 */
@Disabled
@Autonomous(name = "Red Auto Vision", group = "Vision")
public class LinearVisionSample extends LinearVisionOpMode {

    //Frame counter
    int frameCount = 0;

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor spinnerRight;
    DcMotor spinnerLeft;
    DcMotor lift;
    DcMotor shooter;
    Servo tilt;
    Servo push;
    Servo flip;
    OpticalDistanceSensor ods;
    TouchSensor touch;

    DcMotor br, bl, fr, fl;

    ModernRoboticsI2cRangeSensor range;
    LightSensor light;
    ColorSensor color;
    UltrasonicSensor ultra;

    OpenGLMatrix lastLocation = null;

    ArrayList<Double> odsVal = new ArrayList<Double>();

    VuforiaLocalizer vuforia;

    private final int NAVX_DIM_I2C_PORT = 1;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private final double TARGET_ANGLE_DEGREES = -45.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURN_SPEED              = 0.5;
    static final double     WHITE_THRESHOLD = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        /************ Initialize all devices, sensors, IMU and FTCVision. ************/

        /** Initialize Drive **/
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        /** Initialize Lift and Spinner **/
        spinnerRight = hardwareMap.dcMotor.get("rightSpinner");
        spinnerLeft = hardwareMap.dcMotor.get("leftSpinner");
        lift = hardwareMap.dcMotor.get("lift");

        /** Servos and Shooter **/
        tilt = hardwareMap.servo.get("tilt");
        tilt.setPosition(Servo.MAX_POSITION);
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        push = hardwareMap.servo.get("push");
        push.setPosition(Servo.MIN_POSITION);
        flip = hardwareMap.servo.get("flip");
        flip.setPosition(Servo.MAX_POSITION - 0.2);

        /** Initialize Sensors and IMU **/
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        touch = hardwareMap.touchSensor.get("touch");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        navx_device.zeroYaw();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE_REVERSE);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odsVal.add(ods.getLightDetected());

        //Wait for the match to begin
        waitForStart();

        /************ Drive Forward for 12 inches. ************/
        encoderDrive(DRIVE_SPEED, 12, 12, 10);
        odsVal.add(ods.getLightDetected());
        sleep(250);

        /************ Make a 45 degree turn counterclockwise (towards the beacon). ************/
        final double TOTAL_RUN_TIME_SECONDS = 10.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        boolean finishedTurn = false;
        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS && !finishedTurn) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    setDrivePower(0);
                    finishedTurn = true;
                } else {
                    double output = yawPIDResult.getOutput();
                    telemetry.addData("PID Output", output);
                    telemetry.update();
                    if (Math.abs(output) < 0.5){
                        if (output < 0){
                            output = -0.5;
                        }
                        else if (output > 0) {
                            output = 0.5;
                        }
                    }
                    if ( output < 0 ) {
                        /* Rotate Left */
                        backLeftDrive.setPower(-output);
                        frontLeftDrive.setPower(-output);
                        backRightDrive.setPower(output);
                        frontRightDrive.setPower(output);
                    } else {
                        /* Rotate Right */
                        backLeftDrive.setPower(output);
                        frontLeftDrive.setPower(output);
                        backRightDrive.setPower(-output);
                        frontRightDrive.setPower(-output);
                    }
                }
            } else {
			          /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }

        navx_device.close();
        odsVal.add(ods.getLightDetected());
        sleep(250);

        /************ Forward 18 inches. ************/
        encoderDrive(DRIVE_SPEED, 14, 14, 10);
        odsVal.add(ods.getLightDetected());
        sleep(250);

        /************  Flip out cap ball holder so ball doesn't hit it. ************/
        flip.setPosition(Servo.MIN_POSITION);
        odsVal.add(ods.getLightDetected());
        sleep(500);

        /************ Shoot! ************/
        shooter.setPower(1);
        sleep(1500);
        shooter.setPower(0);
        odsVal.add(ods.getLightDetected());
        sleep(250);

        /************ Forward 18 inches again to get in vicinity of white line/beacon. ************/
        encoderDrive(DRIVE_SPEED, 22, 22, 10);
        odsVal.add(ods.getLightDetected());
        sleep(250);


        /************ Forward till the white line ************/
        setDrivePower(DRIVE_SPEED / 2);
        odsVal.add(ods.getLightDetected());
        sleep(250);
        while (opModeIsActive() && (ods.getLightDetected() < WHITE_THRESHOLD)){
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  ods.getLightDetected());
            telemetry.update();
            waitOneFullHardwareCycle();
        }
        setDrivePower(0);
        sleep(250);

        /************ A little bit of forward offset to help line up the camera post-turn. ************/
//        encoderDrive(DRIVE_SPEED / 2, 3, 3, 5);


//        backLeftDrive.setPower(0.4);
//        frontLeftDrive.setPower(0.4);
//        backRightDrive.setPower(-0.4);
//        frontRightDrive.setPower(-0.4);
//
//        while (opModeIsActive() && (ods.getLightDetected() < WHITE_THRESHOLD)){
//            // Display the light level while we are looking for the line
//            telemetry.addData("Light Level",  ods.getLightDetected());
//            telemetry.update();
//            waitOneFullHardwareCycle();
//        }

//        setDrivePower(0);
//        sleep(250);
//        setDrivePower(-0.6);
//
//        while (opModeIsActive() && (range.getDistance(DistanceUnit.MM) > 80)){
//            telemetry.addData("Distance: ",  range.getDistance(DistanceUnit.MM));
//            telemetry.update();
//            waitOneFullHardwareCycle();
//        }
//
//        setDrivePower(0);



        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        navx_device.zeroYaw();

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        yawPIDResult = new navXPIDController.PIDResult();
        finishedTurn = false;
        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS && !finishedTurn) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    setDrivePower(0);
                    finishedTurn = true;
                } else {
                    double output = yawPIDResult.getOutput();
                    telemetry.addData("PID Output", output);
                    telemetry.update();
                    if (Math.abs(output) < 0.5){
                        if (output < 0){
                            output = -0.5;
                        }
                        else if (output > 0) {
                            output = 0.5;
                        }
                    }
                    if ( output < 0 ) {
                        /* Rotate Left */
                        backLeftDrive.setPower(-output);
                        frontLeftDrive.setPower(-output);
                        backRightDrive.setPower(output);
                        frontRightDrive.setPower(output);
                    } else {
                        /* Rotate Right */
                        backLeftDrive.setPower(output);
                        frontLeftDrive.setPower(output);
                        backRightDrive.setPower(-output);
                        frontRightDrive.setPower(-output);
                    }
                }
            } else {
			          /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }

        navx_device.close();

        boolean detectedBeacon = false;

        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed

        int correctTimes = 0;
        while (opModeIsActive()) {
            //Log a few things
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
            telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
            telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
            telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
            telemetry.addData("Frame Counter", frameCount);
            telemetry.update();

//            if (beacon.getAnalysis().getConfidence() > 0.90) {
//                correctTimes += 1;
//            }
            //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
            //Vision will run asynchronously (parallel) to any user code so your programs won't hang
            //You can use hasNewFrame() to test whether vision processed a new frame
            //Once you copy the frame, discard it immediately with discardFrame()
            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                Mat gray = getFrameGray();

                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
                //For this demo, let's just add to a frame counter
                frameCount++;
            }

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }
    }

    public void setDrivePower(double p){
        backLeftDrive.setPower(p);
        backRightDrive.setPower(p);
        frontLeftDrive.setPower(p);
        frontRightDrive.setPower(p);
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backLeftDrive.setTargetPosition(newLeftTarget);
            backRightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            frontLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        backLeftDrive.getCurrentPosition(),
                        backRightDrive.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                waitOneFullHardwareCycle();
            }

            // Stop all motion;
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            frontLeftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
