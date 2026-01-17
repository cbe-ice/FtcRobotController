package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.annotation.SuppressLint;
import java.util.List;

/**
 * OpMode for testing the Webcam and AprilTag detection.
 * NOTE: Although located in the mechanisms package, this is a functional
 * OpMode.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Webcam Test", group = "Test")
public class Webcam extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    /**
     * Main OpMode loop.
     * Initializes AprilTag detection and displays telemetry.
     */
    @Override
    public void runOpMode() {

        initAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();
                telemetry.update();

                if (gamepad1.dpad_down) {

                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {

                    visionPortal.resumeStreaming();
                }

                sleep(20);
            }
        }
        visionPortal.close();

    }

    /**
     * Initializes the AprilTag processor and VisionPortal.
     */
    private void initAprilTag() {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {

            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
        } else {

            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    /**
     * Displays telemetry data for detected AprilTags.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
                        detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch,
                        detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
                        detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {

                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(
                        String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
