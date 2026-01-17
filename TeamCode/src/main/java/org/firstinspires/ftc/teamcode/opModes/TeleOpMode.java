package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DoubleFlywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.vision.VisionData;

/**
 * Main TeleOp control mode.
 * Controls:
 * - Left Stick: Movement (Field Centric)
 * - Right Stick X: Rotation
 * - Left Stick Button: Slow Mode (Hold)
 * - X: Stop Intake (Hold) - NOTE: Intake runs by default?
 * - Y: Run Loader
 * - Right Trigger: Shoot
 */
@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class TeleOpMode extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    DoubleFlywheel shoot = new DoubleFlywheel();
    Limelight limelight = new Limelight();
    GoalTargeter goalTargeter;

    double forward, strafe, rotate;
    double speedMultiplier;
    boolean speedSwitch, intakeSwitch, loadSwitch, unloadSwitch, emergencySwitch;
    float shootSwitch;

    // Auto-aim state
    boolean autoAimEnabled = false;
    boolean lastRightBumper = false;

    @Override
    public void init() {

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shoot.init(hardwareMap);
        limelight.init(hardwareMap);

        telemetry.addData("Left Stick", " Movement");
        telemetry.addData("Left Stick Down", " Speed Switch");
        telemetry.addData("Right Stick", " Rotation");
        telemetry.addData("Right Trigger", " Shoot");
        telemetry.addData("Button X", " Intake Switch");
        telemetry.addData("Button Y", " Load Switch");
        telemetry.addData("Right Bumper", " Toggle Auto-Aim");
        telemetry.addData("D-Pad Up/Down", " Switch Pipeline");

        goalTargeter = new GoalTargeter(limelight);

    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
        speedSwitch = gamepad1.left_stick_button;
        intakeSwitch = gamepad1.x;
        loadSwitch = gamepad1.a;
        unloadSwitch = gamepad1.y;
        shootSwitch = gamepad1.right_trigger;
        emergencySwitch = gamepad1.b;

        // Auto-aim Toggle
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !lastRightBumper) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastRightBumper = rightBumper;

        goalTargeter.update();

        if (speedSwitch) {
            speedMultiplier = 0.5;
        } else {
            speedMultiplier = 1;
        }

        // Intake Control
        // Logic: Runs by default (1), stops when X is pressed (0)
        // TODO: Verify if this inverted logic is intended
        if (intakeSwitch) {
            intake.intake(0);
        } else {
            intake.intake(1);
        }

        if (loadSwitch) {
            intake.load(0.5, 1,0);
        } else if(unloadSwitch) {
            intake.load(-0.5, 1, 0);
        } else {
            intake.load(0.05, 1, 0);
        }

        if (shootSwitch > 0) {
            shoot.shoot(0.355);
        } else {
            shoot.shoot(0);
        }

        if (emergencySwitch) {
            // Emergency Stop
            drive.drive(0, 0, 0);
            intake.stopAll();
            shoot.shoot(0);
            return; // Skip other controls
        }

        if (autoAimEnabled && goalTargeter.hasTarget()) {
            // Override controls with auto-aim
            double drivePower = goalTargeter.getDriveCorrection();
            double turnPower = goalTargeter.getSteeringCorrection();
            // Strafe is manual override, or 0
            double strafePower = strafe * speedMultiplier;

            drive.drive(drivePower, strafePower, turnPower);

            telemetry.addData("Auto-Aim", "ACTIVE");
        } else {
            drive.drive(forward * speedMultiplier, strafe * speedMultiplier, rotate * speedMultiplier);
            telemetry.addData("Auto-Aim", autoAimEnabled ? "SEARCHING" : "OFF");
        }

        // Pipeline Control
        if (gamepad1.dpad_up) {
            limelight.switchPipeline(Limelight.PIPELINE_APRILTAG);
        } else if (gamepad1.dpad_down) {
            limelight.switchPipeline(Limelight.PIPELINE_GREEN);
        } else if (gamepad1.dpad_left) {
            limelight.switchPipeline(Limelight.PIPELINE_PURPLE);
        }

        // Pipeline Name
        String pipelineName;
        switch (limelight.getCurrentPipeline()) {
            case Limelight.PIPELINE_APRILTAG:
                pipelineName = "AprilTag";
                break;
            case Limelight.PIPELINE_GREEN:
                pipelineName = "GREEN";
                break;
            case Limelight.PIPELINE_PURPLE:
                pipelineName = "PURPLE";
                break;
            default:
                pipelineName = "Pipeline " + limelight.getCurrentPipeline();
        }
        telemetry.addData("Pipeline", pipelineName);

        // Vision Telemetry
        VisionData visionData = goalTargeter.getVisionData();
        telemetry.addData("Target", visionData.hasTarget() ? "DETECTED" : "---");
        telemetry.addData("TX/TY/TA", "%.1f° / %.1f° / %.1f%%",
                limelight.getTx(), limelight.getTy(), limelight.getTa());

        // Color Detection
        if (visionData.isGreen()) {
            telemetry.addData("Artifact", "🟢 GREEN");
        } else if (visionData.isPurple()) {
            telemetry.addData("Artifact", "🟣 PURPLE");
        }

        telemetry.update();
    }
}