package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DoubleFlywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

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

    double forward, strafe, rotate;
    double speedMultiplier;
    boolean speedSwitch, intakeSwitch, loadSwitch;
    float shootSwitch;

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
        telemetry.addData("D-Pad Up/Down", " Switch Pipeline");

    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
        speedSwitch = gamepad1.left_stick_button;
        intakeSwitch = gamepad1.x;
        loadSwitch = gamepad1.y;
        shootSwitch = gamepad1.right_trigger;

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
            intake.load(1, 2000, 1000);
        } else {
            intake.load(0, 0, 0);
        }

        if (shootSwitch > 0) {
            shoot.shoot(0.355);
        } else {
            shoot.shoot(0);
        }

        if (gamepad1.b) {
            // Emergency Stop
            drive.drive(0, 0, 0);
            intake.stopAll();
            shoot.shoot(0);
            return; // Skip other controls
        }

        drive.driveFieldRelative(forward * speedMultiplier, strafe * speedMultiplier, rotate * speedMultiplier);

        // Limelight Control & Telemetry
        if (gamepad1.dpad_up) {
            limelight.switchPipeline(0); // Example: Pipeline 0 for AprilTags
        } else if (gamepad1.dpad_down) {
            limelight.switchPipeline(1); // Example: Pipeline 1 for Neural Network
        }

        telemetry.addData("Limelight TX", limelight.getTx());
        telemetry.addData("Limelight TY", limelight.getTy());
        telemetry.addData("Limelight TA", limelight.getTa());

    }
}