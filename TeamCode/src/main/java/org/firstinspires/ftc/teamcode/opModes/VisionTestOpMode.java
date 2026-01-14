package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.vision.MotifDetector;
import org.firstinspires.ftc.teamcode.vision.VisionData;

/**
 * Test OpMode for the vision pipeline.
 * Demonstrates GoalTargeter auto-alignment and MotifDetector pattern
 * recognition.
 *
 * Controls:
 * - Left Stick: Manual driving (when not auto-aligning)
 * - Right Stick X: Rotation
 * - A: Toggle auto-alignment mode
 * - D-Pad Up: Switch to AprilTag pipeline (0)
 * - D-Pad Down: Switch to color pipeline (1)
 * - D-Pad Left: Switch to neural network pipeline (2)
 */
@TeleOp(name = "Vision Test", group = "Test")
public class VisionTestOpMode extends OpMode {

    private MecanumDrive drive;
    private Limelight limelight;
    private GoalTargeter goalTargeter;
    private MotifDetector motifDetector;

    private boolean autoAlignEnabled = false;
    private boolean lastAButton = false;

    @Override
    public void init() {
        // Initialize subsystems
        drive = new MecanumDrive();
        limelight = new Limelight();

        drive.init(hardwareMap);
        limelight.init(hardwareMap);

        // Initialize vision processors
        goalTargeter = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Press A to toggle auto-align");
        telemetry.addData("", "D-Pad to switch pipelines");
    }

    @Override
    public void loop() {
        // Update vision systems
        goalTargeter.update();

        // Update motif detector with latest vision data
        VisionData visionData = goalTargeter.getVisionData();
        motifDetector.update(visionData);

        // Toggle auto-align with A button (rising edge)
        boolean aButton = gamepad1.a;
        if (aButton && !lastAButton) {
            autoAlignEnabled = !autoAlignEnabled;
        }
        lastAButton = aButton;

        // Pipeline switching
        if (gamepad1.dpad_up) {
            goalTargeter.setPipeline(0); // AprilTags
        } else if (gamepad1.dpad_down) {
            goalTargeter.setPipeline(1); // Color tracking
        } else if (gamepad1.dpad_left) {
            goalTargeter.setPipeline(2); // Neural network
        }

        // Driving logic
        double forward, strafe, rotate;

        if (autoAlignEnabled && goalTargeter.hasTarget()) {
            // Auto-align mode: use vision corrections
            forward = goalTargeter.getDriveCorrection();
            strafe = 0;
            rotate = goalTargeter.getSteeringCorrection();
        } else {
            // Manual mode: use gamepad
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
        }

        drive.driveFieldRelative(forward, strafe, rotate);

        // Telemetry
        telemetry.addData("Mode", autoAlignEnabled ? "AUTO-ALIGN" : "MANUAL");
        telemetry.addLine();

        // GoalTargeter data
        telemetry.addData("Target", goalTargeter.hasTarget() ? "DETECTED" : "---");
        if (goalTargeter.hasTarget()) {
            telemetry.addData("  TX (horiz)", "%.1f°", goalTargeter.getTx());
            telemetry.addData("  TY (vert)", "%.1f°", goalTargeter.getTy());
            telemetry.addData("  TA (area)", "%.2f%%", goalTargeter.getTa());
            telemetry.addData("  Locked", goalTargeter.isLocked() ? "YES ✓" : "NO");
        }
        telemetry.addLine();

        // MotifDetector data
        telemetry.addData("Motif", motifDetector.getDetectedMotif().toString());
        if (motifDetector.hasConfidentDetection()) {
            telemetry.addData("  Pattern", motifDetector.getPatternDescription());
            telemetry.addData("  Confidence", "%d detections", motifDetector.getConsecutiveDetections());
        }
        telemetry.addLine();

        // Drive corrections (for debugging)
        if (autoAlignEnabled) {
            telemetry.addData("Steer Correction", "%.2f", goalTargeter.getSteeringCorrection());
            telemetry.addData("Drive Correction", "%.2f", goalTargeter.getDriveCorrection());
        }
    }

    @Override
    public void stop() {
        drive.drive(0, 0, 0);
    }
}
