package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.DoubleFlywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.vision.MotifDetector;
import org.firstinspires.ftc.teamcode.vision.VisionData;

/**
 * Vision and Odometry-enhanced Autonomous OpMode.
 * Uses Limelight for target detection and odometry for precise navigation.
 *
 * Autonomous sequence:
 * 1. Detect MOTIF pattern from OBELISK AprilTags
 * 2. Navigate to scoring position using odometry
 * 3. Fine-align to goal using vision feedback
 * 4. Execute scoring action
 */
@Autonomous(name = "Vision Auto", group = "Auto")
public class AutonomousMode extends LinearOpMode {

    // Subsystems
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private Limelight limelight;
    private Intake intake;
    private DoubleFlywheel shooter;
    private GoalTargeter goalTargeter;
    private MotifDetector motifDetector;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    // State machine
    private enum AutoState {
        INIT,
        SCAN_FOR_MOTIF,
        NAVIGATE_TO_TARGET,
        SCAN_FOR_ARTIFACT,
        ALIGN_TO_GOAL,
        EXECUTE_ACTION,
        NAVIGATE_TO_PICKUP,
        PICKUP_BALL,
        DONE
    }

    private AutoState currentState = AutoState.INIT;

    // Configuration
    private static final double SCAN_TIMEOUT_SEC = 3.0;
    private static final double NAV_TIMEOUT_SEC = 8.0;
    private static final double ARTIFACT_SCAN_TIMEOUT_SEC = 2.0;
    private static final double ALIGN_TIMEOUT_SEC = 5.0;
    private static final double VISION_LOSS_TIMEOUT_SEC = 1.5;
    private static final double SHOOT_SPEED = 0.55; // Adjusted for autonomous

    // Detected artifact type
    private boolean targetIsGreen = false;
    private boolean targetIsPurple = false;
    private double visionLostTime = 0;

    // 3-ball auto tracking
    private int ballsScored = 0;
    private static final int TARGET_BALLS = 3;

    // Odometry control gains
    private static final double DRIVE_KP = 0.05;
    private static final double STRAFE_KP = 0.05;
    private static final double ROTATION_KP = 0.8;
    private static final double MAX_AUTO_POWER = 0.5;
    private static final double POSITION_TOLERANCE = 2.0; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(5); // radians

    // Target waypoints (adjust for your field layout)
    private static final double[][] MOTIF_TARGETS = {
            { 36.0, 0.0, 0.0 }, // GPP target: x, y, heading
            { 36.0, 12.0, 0.0 }, // PGP target
            { 36.0, -12.0, 0.0 } // PPG target
    };
    private static final double[] DEFAULT_TARGET = { 36.0, 0.0, 0.0 };

    // Scoring and pickup positions (calibrate these for your field)
    private static final double[] SCORING_POSITION = { 36.0, 0.0, 0.0 }; // Where to shoot from
    private static final double[] PICKUP_POSITION = { 12.0, 0.0, Math.toRadians(180) }; // Ball pickup location
    private static final double PICKUP_TIMEOUT_SEC = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initHardware();

        // Wait for start with status updates
        while (!isStarted() && !isStopRequested()) {
            // Pre-start scanning
            goalTargeter.update();
            VisionData visionData = goalTargeter.getVisionData();
            motifDetector.update(visionData);

            telemetry.addData("Status", "Initialized - Waiting for Start");
            telemetry.addData("Limelight", goalTargeter.hasTarget() ? "Target Detected" : "No Target");
            telemetry.addData("Motif", motifDetector.getDetectedMotif().toString());
            telemetry.addData("Odometry", odometry.getTelemetryString());
            telemetry.update();
        }

        // Start autonomous
        runtime.reset();
        odometry.reset();
        currentState = AutoState.SCAN_FOR_MOTIF;
        stateTimer.reset();

        while (opModeIsActive()) {
            // Update systems every loop
            odometry.update();
            goalTargeter.update();
            VisionData visionData = goalTargeter.getVisionData();
            motifDetector.update(visionData);

            // Run state machine
            switch (currentState) {
                case SCAN_FOR_MOTIF:
                    runScanForMotif();
                    break;

                case NAVIGATE_TO_TARGET:
                    runNavigateToTarget();
                    break;

                case ALIGN_TO_GOAL:
                    runAlignToGoal();
                    break;

                case EXECUTE_ACTION:
                    runExecuteAction();
                    break;

                case SCAN_FOR_ARTIFACT:
                    runScanForArtifact();
                    break;

                case NAVIGATE_TO_PICKUP:
                    runNavigateToPickup();
                    break;

                case PICKUP_BALL:
                    runPickupBall();
                    break;

                case DONE:
                    drive.drive(0, 0, 0);
                    intake.stopAll();
                    shooter.shoot(0);
                    break;
            }

            // Telemetry
            telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
            telemetry.addData("State", currentState.toString());
            telemetry.addLine();
            telemetry.addData("Odometry", odometry.getTelemetryString());
            telemetry.addData("Motif", motifDetector.getDetectedMotif().toString());
            telemetry.addLine();
            telemetry.addData("Target", goalTargeter.hasTarget() ? "YES" : "NO");
            telemetry.addData("Balls Scored", ballsScored + "/" + TARGET_BALLS);
            if (goalTargeter.hasTarget()) {
                telemetry.addData("  TX", "%.1f°", goalTargeter.getTx());
                telemetry.addData("  Locked", goalTargeter.isLocked() ? "YES" : "NO");
            }
            telemetry.update();
        }
    }

    /**
     * Initialize all hardware subsystems.
     */
    private void initHardware() {
        drive = new MecanumDrive();
        odometry = new OdometrySubsystem();
        limelight = new Limelight();

        drive.init(hardwareMap);
        odometry.init(hardwareMap);
        limelight.init(hardwareMap);
        intake = new Intake();
        shooter = new DoubleFlywheel();
        intake.init(hardwareMap);
        shooter.init(hardwareMap);

        // Set Limelight to AprilTag pipeline for MOTIF detection
        limelight.switchPipeline(0);

        goalTargeter = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
    }

    /**
     * State: Scan for MOTIF pattern.
     * Slowly rotates while looking for OBELISK AprilTags.
     */
    private void runScanForMotif() {
        if (motifDetector.hasConfidentDetection()) {
            // Pattern detected, set target based on MOTIF
            MotifDetector.Motif motif = motifDetector.getDetectedMotif();
            setTargetForMotif(motif);
            drive.drive(0, 0, 0);
            transitionTo(AutoState.NAVIGATE_TO_TARGET);
            return;
        }

        if (stateTimer.seconds() > SCAN_TIMEOUT_SEC) {
            // Timeout - use default target
            telemetry.addData("Warning", "MOTIF scan timeout - using default");
            odometry.setTarget(DEFAULT_TARGET[0], DEFAULT_TARGET[1], DEFAULT_TARGET[2]);
            transitionTo(AutoState.NAVIGATE_TO_TARGET);
            return;
        }

        // Slow rotation to scan for tags
        drive.drive(0, 0, 0.2);
    }

    /**
     * Sets the navigation target based on detected MOTIF.
     */
    private void setTargetForMotif(MotifDetector.Motif motif) {
        double[] target;
        switch (motif) {
            case GPP:
                target = MOTIF_TARGETS[0];
                break;
            case PGP:
                target = MOTIF_TARGETS[1];
                break;
            case PPG:
                target = MOTIF_TARGETS[2];
                break;
            default:
                target = DEFAULT_TARGET;
        }
        odometry.setTarget(target[0], target[1], target[2]);
    }

    /**
     * State: Navigate to target using odometry.
     */
    private void runNavigateToTarget() {
        if (odometry.isAtTarget(POSITION_TOLERANCE, HEADING_TOLERANCE)) {
            // Reached target position
            drive.drive(0, 0, 0);
            // Switch to color pipeline for artifact scanning
            limelight.switchPipeline(Limelight.PIPELINE_GREEN);
            transitionTo(AutoState.SCAN_FOR_ARTIFACT);
            return;
        }

        if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            // Timeout - proceed to artifact scan anyway
            telemetry.addData("Warning", "Navigation timeout");
            limelight.switchPipeline(Limelight.PIPELINE_GREEN);
            transitionTo(AutoState.SCAN_FOR_ARTIFACT);
            return;
        }

        // Use odometry-based proportional control
        double forward = odometry.getDriveCorrection(DRIVE_KP, MAX_AUTO_POWER);
        double strafe = odometry.getStrafeCorrection(STRAFE_KP, MAX_AUTO_POWER);
        double rotate = odometry.getRotationCorrection(ROTATION_KP, MAX_AUTO_POWER);

        drive.driveFieldRelative(forward, strafe, rotate);
    }

    /**
     * State: Fine-align to goal using vision feedback.
     * Includes vision loss fallback.
     */
    private void runAlignToGoal() {
        VisionData visionData = goalTargeter.getVisionData();

        if (goalTargeter.isLocked()) {
            // Locked on target
            drive.drive(0, 0, 0);
            visionLostTime = 0;
            transitionTo(AutoState.EXECUTE_ACTION);
            return;
        }

        if (stateTimer.seconds() > ALIGN_TIMEOUT_SEC) {
            // Timeout - proceed anyway
            telemetry.addData("Warning", "Align timeout - proceeding");
            transitionTo(AutoState.EXECUTE_ACTION);
            return;
        }

        if (goalTargeter.hasTarget()) {
            // Reset vision lost timer
            visionLostTime = 0;
            // Use vision-based corrections with field-relative control
            double steer = goalTargeter.getSteeringCorrection();
            double forward = goalTargeter.getDriveCorrection();
            drive.driveFieldRelative(forward, 0, steer);
        } else {
            // Vision lost - track time
            if (visionLostTime == 0) {
                visionLostTime = runtime.seconds();
            }

            // Check for vision loss timeout
            if (runtime.seconds() - visionLostTime > VISION_LOSS_TIMEOUT_SEC) {
                telemetry.addData("Warning", "Vision lost - proceeding with last known position");
                drive.drive(0, 0, 0);
                transitionTo(AutoState.EXECUTE_ACTION);
                return;
            }

            // No target - slow search rotation (field-relative for consistency)
            drive.driveFieldRelative(0, 0, 0.15);
        }
    }

    /**
     * State: Scan for colored artifacts.
     * Switches to color pipelines to detect green/purple.
     */
    private void runScanForArtifact() {
        VisionData visionData = goalTargeter.getVisionData();

        // Check for color detection
        if (visionData.hasTarget()) {
            if (visionData.isGreen()) {
                targetIsGreen = true;
                targetIsPurple = false;
                telemetry.addData("Artifact", "GREEN detected");
                transitionTo(AutoState.ALIGN_TO_GOAL);
                return;
            } else if (visionData.isPurple()) {
                targetIsGreen = false;
                targetIsPurple = true;
                telemetry.addData("Artifact", "PURPLE detected");
                transitionTo(AutoState.ALIGN_TO_GOAL);
                return;
            }
        }

        if (stateTimer.seconds() > ARTIFACT_SCAN_TIMEOUT_SEC) {
            // Timeout - proceed without color info
            telemetry.addData("Warning", "Artifact scan timeout");
            transitionTo(AutoState.ALIGN_TO_GOAL);
            return;
        }

        // Alternate between green and purple pipelines
        if ((int) (stateTimer.seconds() * 2) % 2 == 0) {
            limelight.switchPipeline(Limelight.PIPELINE_GREEN);
        } else {
            limelight.switchPipeline(Limelight.PIPELINE_PURPLE);
        }
    }

    /**
     * State: Execute the scoring action.
     * Shoots all 3 preloaded balls in sequence with delays.
     * Timing: spin up (1s) -> shoot ball 1 (1s) -> delay (0.5s) -> shoot ball 2 (1s) -> delay (0.5s) -> shoot ball 3 (1s) -> done
     */
    private void runExecuteAction() {
        double time = stateTimer.seconds();
        
        // Keep shooter spinning throughout
        shooter.shoot(SHOOT_SPEED);
        
        // Timeline for 3 balls:
        // 0.0 - 1.0: Spin up (no feeding)
        // 1.0 - 2.0: Feed ball 1
        // 2.0 - 2.5: Pause
        // 2.5 - 3.5: Feed ball 2
        // 3.5 - 4.0: Pause
        // 4.0 - 5.0: Feed ball 3
        // 5.0+: Done
        
        if (time < 1.0) {
            // Spin up shooter
            intake.stopAll();
        } else if (time < 2.0) {
            // Feed ball 1
            intake.load(1.0, 100, 0);
        } else if (time < 2.5) {
            // Pause between shots
            intake.stopAll();
        } else if (time < 3.5) {
            // Feed ball 2
            intake.load(1.0, 100, 0);
        } else if (time < 4.0) {
            // Pause between shots
            intake.stopAll();
        } else if (time < 5.0) {
            // Feed ball 3
            intake.load(1.0, 100, 0);
        } else {
            // All 3 balls shot - done!
            shooter.shoot(0);
            intake.stopAll();
            ballsScored = TARGET_BALLS;
            transitionTo(AutoState.DONE);
        }
    }

    /**
     * State: Navigate back to pickup location.
     */
    private void runNavigateToPickup() {
        odometry.setTarget(PICKUP_POSITION[0], PICKUP_POSITION[1], PICKUP_POSITION[2]);

        if (odometry.isAtTarget(POSITION_TOLERANCE, HEADING_TOLERANCE)) {
            drive.drive(0, 0, 0);
            transitionTo(AutoState.PICKUP_BALL);
            return;
        }

        if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            telemetry.addData("Warning", "Pickup navigation timeout");
            transitionTo(AutoState.PICKUP_BALL);
            return;
        }

        double forward = odometry.getDriveCorrection(DRIVE_KP, MAX_AUTO_POWER);
        double strafe = odometry.getStrafeCorrection(STRAFE_KP, MAX_AUTO_POWER);
        double rotate = odometry.getRotationCorrection(ROTATION_KP, MAX_AUTO_POWER);

        drive.driveFieldRelative(forward, strafe, rotate);
    }

    /**
     * State: Pick up a ball using the intake.
     */
    private void runPickupBall() {
        // Run intake to pick up ball
        intake.intake(1.0);

        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            // Assume ball is loaded, head back to scoring position
            intake.intake(0);
            odometry.setTarget(SCORING_POSITION[0], SCORING_POSITION[1], SCORING_POSITION[2]);
            transitionTo(AutoState.NAVIGATE_TO_TARGET);
            return;
        }

        // Slow forward creep while intaking
        drive.driveFieldRelative(0.15, 0, 0);
    }

    /**
     * Transition to a new state and reset the state timer.
     */
    private void transitionTo(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }
}
