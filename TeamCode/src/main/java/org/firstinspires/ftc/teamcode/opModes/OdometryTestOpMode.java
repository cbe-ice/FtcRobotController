package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.OdometrySubsystem;

/**
 * Test OpMode for the odometry subsystem.
 * Drive the robot around and watch the position update in real-time.
 *
 * Controls:
 * - Left Stick: Movement (field-centric)
 * - Right Stick X: Rotation
 * - A: Reset odometry to origin
 * - B: Set target to current position + 24 inches forward
 * - X: Toggle auto-drive to target
 * - Y: Set target to origin (0, 0)
 */
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTestOpMode extends OpMode {

    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private ElapsedTime loopTimer;

    private boolean autoDriveEnabled = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastXButton = false;
    private boolean lastYButton = false;

    // Control gains
    private static final double DRIVE_KP = 0.05;
    private static final double STRAFE_KP = 0.05;
    private static final double ROTATION_KP = 0.8;
    private static final double MAX_AUTO_POWER = 0.5;
    private static final double POSITION_TOLERANCE = 1.0; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(3); // radians

    @Override
    public void init() {
        drive = new MecanumDrive();
        odometry = new OdometrySubsystem();
        loopTimer = new ElapsedTime();

        drive.init(hardwareMap);
        odometry.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Reset, B=+24in, X=AutoDrive, Y=GoHome");
    }

    @Override
    public void loop() {
        loopTimer.reset();

        // Update odometry
        odometry.update();

        // Button handling (rising edge detection)
        boolean aButton = gamepad1.a;
        boolean bButton = gamepad1.b;
        boolean xButton = gamepad1.x;
        boolean yButton = gamepad1.y;

        // A: Reset odometry
        if (aButton && !lastAButton) {
            odometry.reset();
            autoDriveEnabled = false;
        }

        // B: Set target 24 inches forward from current position
        if (bButton && !lastBButton) {
            double targetX = odometry.getX() + 24.0 * Math.cos(odometry.getHeading());
            double targetY = odometry.getY() + 24.0 * Math.sin(odometry.getHeading());
            odometry.setTarget(targetX, targetY, odometry.getHeading());
        }

        // X: Toggle auto-drive
        if (xButton && !lastXButton) {
            autoDriveEnabled = !autoDriveEnabled;
        }

        // Y: Set target to origin
        if (yButton && !lastYButton) {
            odometry.setTarget(0, 0, 0);
        }

        lastAButton = aButton;
        lastBButton = bButton;
        lastXButton = xButton;
        lastYButton = yButton;

        // Driving logic
        double forward, strafe, rotate;

        if (autoDriveEnabled) {
            if (odometry.isAtTarget(POSITION_TOLERANCE, HEADING_TOLERANCE)) {
                // At target - stop
                forward = 0;
                strafe = 0;
                rotate = 0;
                autoDriveEnabled = false;
            } else {
                // Auto-drive to target
                forward = odometry.getDriveCorrection(DRIVE_KP, MAX_AUTO_POWER);
                strafe = odometry.getStrafeCorrection(STRAFE_KP, MAX_AUTO_POWER);
                rotate = odometry.getRotationCorrection(ROTATION_KP, MAX_AUTO_POWER);
            }
        } else {
            // Manual control
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
        }

        drive.driveFieldRelative(forward, strafe, rotate);

        // Telemetry
        telemetry.addData("Mode", autoDriveEnabled ? "AUTO-DRIVE" : "MANUAL");
        telemetry.addLine();

        telemetry.addData("Position X", "%.2f inches", odometry.getX());
        telemetry.addData("Position Y", "%.2f inches", odometry.getY());
        telemetry.addData("Heading", "%.1f degrees", odometry.getHeadingDegrees());
        telemetry.addLine();

        telemetry.addData("Distance to Target", "%.2f inches", odometry.getDistanceToTarget());
        telemetry.addData("At Target", odometry.isAtTarget(POSITION_TOLERANCE, HEADING_TOLERANCE) ? "YES ✓" : "NO");
        telemetry.addLine();

        if (autoDriveEnabled) {
            telemetry.addData("Drive Power", "%.2f", forward);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Rotate Power", "%.2f", rotate);
        }

        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());
    }

    @Override
    public void stop() {
        drive.drive(0, 0, 0);
    }
}
