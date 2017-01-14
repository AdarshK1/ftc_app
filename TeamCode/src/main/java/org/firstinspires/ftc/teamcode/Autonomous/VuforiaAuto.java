package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by akulkarni on 12/30/16.
 */

@Autonomous(name = "VuforiaAuto")
public class VuforiaAuto extends LinearOpMode
{
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AV/at7D/////AAAAGeh38frwBEGUke6F4wbsl2eI/QsNUvPFSZZGsy3+2Tifa5qSfnCh93gmS0KSfS526VeacacNr5M3kk64htoLkR0k4nIyccwt4vAvID76fniTyv5ykj7AFVdPdm3HRf8dn4Kd/MYmsVCnoKeklJvUlPkRBf6W1vBa63dF75Fc8H15e9+s5q3PHaz/jrrdzVaXm4yZB0f/vBmsA1kw8ERWrPhD1ZYP4T2mpzpRAQvxNTBBc9yNzSQ8kbEm6a0SN8qviw8EQofAzrtL5iwlF8V0e21Ldjn5SCh9qRcn0LBz6olZYHU+yjPB6qlabBFpM76eEUgUMeE8CiyTVK0SkB006QJKSHyvMQQd7+ds+LMztKoe";

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException
    {
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");

        setupVuforia();

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        visionTargets.activate();

        while(opModeIsActive())
        {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            telemetry.update();
            idle();
        }
    }

    public void setupVuforia()
    {
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC 2016-17");

        target = visionTargets.get(0);
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        phoneLocation = createMatrix(0, 255, 0, 90, 0, 0);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix) { return matrix.formatAsTransform(); }
}
