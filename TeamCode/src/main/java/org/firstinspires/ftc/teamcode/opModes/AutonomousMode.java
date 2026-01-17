package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanisms.DoubleFlywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.vision.MotifDetector;
import org.firstinspires.ftc.teamcode.vision.VisionData;

/**
 * 6-Ball Autonomous OpMode with Alliance Selection.
 * 
 * Sequence:
 * 1. Detect MOTIF pattern from OBELISK AprilTags
 * 2. Shoot 3 preloaded balls with 0.5s delays
 * 3. Navigate to ball cluster matching MOTIF
 * 4. Pick up 3 balls
 * 5. Navigate to shooting position
 * 6. Align and shoot 3 balls
 * 
 * Uses odometry for primary navigation with vision validation.
 */
@Autonomous(name = "6-Ball Auto", group = "Auto")
public class AutonomousMode extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED, BLUE
    }

    private Alliance selectedAlliance = Alliance.RED;

    // ===================== SUBSYSTEMS =====================
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private Limelight limelight;
    private Intake intake;
    private DoubleFlywheel shooter;
    private GoalTargeter goalTargeter;
    private MotifDetector motifDetector;

    // ===================== TIMING =====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        SCAN_MOTIF,
        SHOOT_PRELOADS,
        NAV_TO_BALLS,
        PICKUP_BALLS,
        NAV_TO_SHOOT,
        ALIGN_AND_SHOOT,
        RECOVERY,
        DONE
    }

    private AutoState currentState = AutoState.INIT;
    private AutoState recoveryReturnState = AutoState.NAV_TO_BALLS;

    // ===================== CONFIGURATION =====================
    // Timeouts
    private static final double SCAN_TIMEOUT_SEC = 3.0;
    private static final double NAV_TIMEOUT_SEC = 8.0;
    private static final double PICKUP_TIMEOUT_SEC = 5.0;
    private static final double ALIGN_TIMEOUT_SEC = 4.0;
    private static final double VISION_LOSS_TIMEOUT_SEC = 1.5;
    private static final double RECOVERY_TIMEOUT_SEC = 3.0;

    // Shooting
    private static final double SHOOT_SPEED = 0.55;
    private static final double SHOT_DURATION_SEC = 0.8;
    private static final double SHOT_DELAY_SEC = 0.5;

    // Navigation tolerances
    private static final double POSITION_TOLERANCE = 3.0; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(5);
    private static final double VISION_VALIDATION_THRESHOLD = 6.0; // inches - triggers recovery

    // Odometry control gains
    private static final double DRIVE_KP = 0.05;
    private static final double STRAFE_KP = 0.05;
    private static final double ROTATION_KP = 0.8;
    private static final double MAX_AUTO_POWER = 0.6;

    // ===================== FIELD COORDINATES (inches) =====================
    // Origin: Bottom-left corner of field
    // Robot starts 1.5ft (18 inches) from goal

    // Blue Alliance ball positions (right side of field)
    private static final double[][] BLUE_BALL_POSITIONS = {
            { 112.5, 3.0, Math.toRadians(0) }, // GPP - green ball position
            { 112.5, 5.0, Math.toRadians(0) }, // PGP - P ball position
            { 112.5, 7.0, Math.toRadians(0) }, // PPG - P ball position
    };

    // Red Alliance ball positions (left side of field)
    private static final double[][] RED_BALL_POSITIONS = {
            { 19.5, 3.0, Math.toRadians(180) }, // GPP - green ball position
            { 19.5, 5.0, Math.toRadians(180) }, // PGP - P ball position
            { 19.5, 7.0, Math.toRadians(180) }, // PPG - P ball position
    };

    // Starting positions (1.5ft = 18 inches from goal)
    private static final double[] BLUE_START = { 112.5 - 18.0, 0.0, Math.toRadians(0) };
    private static final double[] RED_START = { 19.5 + 18.0, 0.0, Math.toRadians(180) };

    // Shooting positions (same as start - shoot from starting position)
    private static final double[] BLUE_SHOOT_POSITION = { 112.5 - 18.0, 0.0, Math.toRadians(0) };
    private static final double[] RED_SHOOT_POSITION = { 19.5 + 18.0, 0.0, Math.toRadians(180) };

    // ===================== STATE VARIABLES =====================
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private double[] currentTarget = new double[3];
    private int ballsShot = 0;
    private int ballsPickedUp = 0;
    private double visionLostTime = 0;
    private boolean inSecondShootingPhase = false;

    // ===================== MAIN LOOP =====================
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initHardware();

        // Alliance selection and pre-match setup
        while (!isStarted() && !isStopRequested()) {
            // Alliance selection with bumpers
            if (gamepad1.left_bumper) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.right_bumper) {
                selectedAlliance = Alliance.BLUE;
            }

            // Pre-start vision scanning
            goalTargeter.update();
            VisionData visionData = goalTargeter.getVisionData();
            motifDetector.update(visionData);

            // Telemetry
            telemetry.addLine("=== 6-BALL AUTONOMOUS ===");
            telemetry.addLine();
            telemetry.addData("Alliance", selectedAlliance.toString());
            telemetry.addLine("  [LB] = RED  |  [RB] = BLUE");
            telemetry.addLine();
            telemetry.addData("Limelight", goalTargeter.hasTarget() ? "Target Detected" : "No Target");
            telemetry.addData("Pre-scan Motif", motifDetector.getDetectedMotif().toString());
            telemetry.update();
        }

        // Set starting position based on alliance
        double[] startPos = (selectedAlliance == Alliance.BLUE) ? BLUE_START : RED_START;
        odometry.setPosition(startPos[0], startPos[1], startPos[2]);

        RobotLog.d("AUTO", "Starting 6-Ball Auto - Alliance: " + selectedAlliance);
        RobotLog.d("AUTO", "Start position: (" + startPos[0] + ", " + startPos[1] + ")");

        // Start autonomous
        runtime.reset();
        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        while (opModeIsActive()) {
            // Update systems every loop
            odometry.update();
            goalTargeter.update();
            VisionData visionData = goalTargeter.getVisionData();
            motifDetector.update(visionData);

            // Run state machine
            switch (currentState) {
                case SCAN_MOTIF:
                    runScanMotif();
                    break;
                case SHOOT_PRELOADS:
                    runShootPreloads();
                    break;
                case NAV_TO_BALLS:
                    runNavToBalls();
                    break;
                case PICKUP_BALLS:
                    runPickupBalls();
                    break;
                case NAV_TO_SHOOT:
                    runNavToShoot();
                    break;
                case ALIGN_AND_SHOOT:
                    runAlignAndShoot();
                    break;
                case RECOVERY:
                    runRecovery();
                    break;
                case DONE:
                    drive.drive(0, 0, 0);
                    intake.stopAll();
                    shooter.shoot(0);
                    break;
            }

            // Telemetry
            updateTelemetry();
        }
    }

    // ===================== INITIALIZATION =====================
    private void initHardware() {
        drive = new MecanumDrive();
        odometry = new OdometrySubsystem();
        limelight = new Limelight();
        intake = new Intake();
        shooter = new DoubleFlywheel();

        drive.init(hardwareMap);
        odometry.init(hardwareMap);
        limelight.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);

        // AprilTag pipeline for MOTIF detection
        limelight.switchPipeline(0);

        goalTargeter = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
    }

    // ===================== STATE: SCAN MOTIF =====================
    private void runScanMotif() {
        if (motifDetector.hasConfidentDetection()) {
            detectedMotif = motifDetector.getDetectedMotif();
            RobotLog.d("AUTO", "Motif detected: " + detectedMotif);
            drive.drive(0, 0, 0);
            transitionTo(AutoState.SHOOT_PRELOADS);
            return;
        }

        if (stateTimer.seconds() > SCAN_TIMEOUT_SEC) {
            // Timeout - use GPP as default
            detectedMotif = MotifDetector.Motif.GPP;
            RobotLog.d("AUTO", "Motif timeout - defaulting to GPP");
            drive.drive(0, 0, 0);
            transitionTo(AutoState.SHOOT_PRELOADS);
            return;
        }

        // Slow rotation to scan for tags
        drive.drive(0, 0, 0.2);
    }

    // ===================== STATE: SHOOT PRELOADS =====================
    private void runShootPreloads() {
        double time = stateTimer.seconds();

        // Keep shooter spinning throughout
        shooter.shoot(SHOOT_SPEED);

        // Calculate timing for 3 shots with delays
        // Pattern: spin-up (1s) -> [shoot (0.8s) -> delay (0.5s)] x 3
        double spinUpTime = 1.0;
        double cycleTime = SHOT_DURATION_SEC + SHOT_DELAY_SEC; // 1.3s per ball

        if (time < spinUpTime) {
            // Spin up
            intake.stopAll();
        } else {
            double shootingTime = time - spinUpTime;
            int currentBall = (int) (shootingTime / cycleTime);
            double timeInCycle = shootingTime % cycleTime;

            if (currentBall < 3) {
                if (timeInCycle < SHOT_DURATION_SEC) {
                    // Feeding ball
                    intake.load(1.0, 100, 0);
                } else {
                    // Delay between shots
                    intake.stopAll();
                }
                ballsShot = currentBall + 1;
            } else {
                // All 3 preloads shot
                shooter.shoot(0);
                intake.stopAll();
                ballsShot = 3;
                RobotLog.d("AUTO", "Preloads complete - 3 balls shot");

                // Set target for ball pickup
                setTargetForMotif();
                transitionTo(AutoState.NAV_TO_BALLS);
            }
        }
    }

    // ===================== STATE: NAVIGATE TO BALLS =====================
    private void runNavToBalls() {
        // Check if at target
        if (odometry.isAtTarget(POSITION_TOLERANCE, HEADING_TOLERANCE)) {
            RobotLog.d("AUTO", "Arrived at ball pickup location");
            drive.drive(0, 0, 0);
            transitionTo(AutoState.PICKUP_BALLS);
            return;
        }

        // Timeout
        if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            RobotLog.d("AUTO", "Nav to balls timeout - proceeding to pickup");
            drive.drive(0, 0, 0);
            transitionTo(AutoState.PICKUP_BALLS);
            return;
        }

        // Vision validation (check if odometry drifted significantly)
        if (goalTargeter.hasTarget() && goalTargeter.getVisionData().hasValidPose()) {
            // Compare vision position with odometry - could trigger recovery
            // For now, just log it
            RobotLog.d("AUTO", "Vision validated at nav point");
        }

        // Proportional control navigation
        double forward = odometry.getDriveCorrection(DRIVE_KP, MAX_AUTO_POWER);
        double strafe = odometry.getStrafeCorrection(STRAFE_KP, MAX_AUTO_POWER);
        double rotate = odometry.getRotationCorrection(ROTATION_KP, MAX_AUTO_POWER);

        drive.driveFieldRelative(forward, strafe, rotate);
    }

    // ===================== STATE: PICKUP BALLS =====================
    private void runPickupBalls() {
        // Run intake continuously
        intake.intake(1.0);

        // Slow forward creep while intaking
        drive.driveFieldRelative(0.15, 0, 0);

        // Time-based pickup (assume 3 balls collected after timeout)
        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            RobotLog.d("AUTO", "Pickup complete - collected balls");
            intake.intake(0);
            drive.drive(0, 0, 0);
            ballsPickedUp = 3;

            // Set target for shooting position
            setShootingTarget();
            transitionTo(AutoState.NAV_TO_SHOOT);
        }
    }

    // ===================== STATE: NAVIGATE TO SHOOT =====================
    private void runNavToShoot() {
        // Check if at target
        if (odometry.isAtTarget(POSITION_TOLERANCE, HEADING_TOLERANCE)) {
            RobotLog.d("AUTO", "Arrived at shooting position");
            drive.drive(0, 0, 0);
            inSecondShootingPhase = true;
            ballsShot = 0; // Reset for second shooting phase
            transitionTo(AutoState.ALIGN_AND_SHOOT);
            return;
        }

        // Timeout
        if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            RobotLog.d("AUTO", "Nav to shoot timeout - proceeding to shoot");
            drive.drive(0, 0, 0);
            inSecondShootingPhase = true;
            ballsShot = 0;
            transitionTo(AutoState.ALIGN_AND_SHOOT);
            return;
        }

        // Proportional control navigation
        double forward = odometry.getDriveCorrection(DRIVE_KP, MAX_AUTO_POWER);
        double strafe = odometry.getStrafeCorrection(STRAFE_KP, MAX_AUTO_POWER);
        double rotate = odometry.getRotationCorrection(ROTATION_KP, MAX_AUTO_POWER);

        drive.driveFieldRelative(forward, strafe, rotate);
    }

    // ===================== STATE: ALIGN AND SHOOT =====================
    private void runAlignAndShoot() {
        double time = stateTimer.seconds();

        // First, try to fine-align with vision (first 2 seconds)
        if (time < 2.0) {
            if (goalTargeter.hasTarget()) {
                visionLostTime = 0;
                double steer = goalTargeter.getSteeringCorrection();
                drive.driveFieldRelative(0, 0, steer);

                if (goalTargeter.isLocked()) {
                    RobotLog.d("AUTO", "Vision lock acquired - shooting");
                    // Skip to shooting immediately
                    stateTimer.reset();
                    // Add 2 seconds to skip alignment phase
                }
            } else {
                // Vision lost tracking
                if (visionLostTime == 0) {
                    visionLostTime = runtime.seconds();
                }

                if (runtime.seconds() - visionLostTime > VISION_LOSS_TIMEOUT_SEC) {
                    RobotLog.d("AUTO", "Vision lost - shooting from current position");
                    // Proceed to shooting
                }

                // Slow search rotation
                drive.driveFieldRelative(0, 0, 0.1);
            }
            return;
        }

        // Shooting phase (after alignment or timeout)
        drive.drive(0, 0, 0);
        shooter.shoot(SHOOT_SPEED);

        double shootTime = time - 2.0; // Offset for alignment phase
        double spinUpTime = 1.0;
        double cycleTime = SHOT_DURATION_SEC + SHOT_DELAY_SEC;

        if (shootTime < spinUpTime) {
            intake.stopAll();
        } else {
            double firingTime = shootTime - spinUpTime;
            int currentBall = (int) (firingTime / cycleTime);
            double timeInCycle = firingTime % cycleTime;

            if (currentBall < 3) {
                if (timeInCycle < SHOT_DURATION_SEC) {
                    intake.load(1.0, 100, 0);
                } else {
                    intake.stopAll();
                }
                ballsShot = currentBall + 1;
            } else {
                // All balls shot - done!
                shooter.shoot(0);
                intake.stopAll();
                ballsShot = 3;
                RobotLog.d("AUTO", "6-Ball Auto COMPLETE!");
                transitionTo(AutoState.DONE);
            }
        }
    }

    private void runRecovery() {
        // Attempt to re-localize using AprilTag
        if (goalTargeter.hasTarget() && goalTargeter.getVisionData().hasValidPose()) { // <-- CHANGED
            // Update odometry from vision
            VisionData visionData = goalTargeter.getVisionData();

            // getBotPose() returns a Pose3D object, not double[]
            Pose3D botPose = visionData.getBotPose();

            if (botPose != null) {
                // Extract position from Pose3D
                // Pose3D uses getPosition() which returns a Position with x, y, z
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;

                // Update odometry with vision position
                // You may need to convert units (Limelight often uses meters)
                // and adjust coordinate system based on your field setup
                odometry.setPosition(x, y, odometry.getHeading());

                RobotLog.d("AUTO", "Recovery: Re-localized from AprilTag at (" + x + ", " + y + ")");
                transitionTo(recoveryReturnState);
                return;
            }
        }

        // Timeout - continue anyway
        if (stateTimer.seconds() > RECOVERY_TIMEOUT_SEC) {
            RobotLog.d("AUTO", "Recovery timeout - continuing with current position");
            transitionTo(recoveryReturnState);
            return;
        }

        // Slow rotation to find AprilTag
        drive.drive(0, 0, 0.15);
    }
    // ===================== HELPER METHODS =====================

    private void setTargetForMotif() {
        double[][] ballPositions = (selectedAlliance == Alliance.BLUE)
                ? BLUE_BALL_POSITIONS
                : RED_BALL_POSITIONS;

        int index;
        switch (detectedMotif) {
            case GPP:
                index = 0;
                break;
            case PGP:
                index = 1;
                break;
            case PPG:
                index = 2;
                break;
            default:
                index = 0; // Default to GPP
        }

        currentTarget = ballPositions[index];
        odometry.setTarget(currentTarget[0], currentTarget[1], currentTarget[2]);
        RobotLog.d("AUTO", "Target set for " + detectedMotif + ": (" +
                currentTarget[0] + ", " + currentTarget[1] + ")");
    }

    private void setShootingTarget() {
        double[] shootPos = (selectedAlliance == Alliance.BLUE)
                ? BLUE_SHOOT_POSITION
                : RED_SHOOT_POSITION;

        currentTarget = shootPos;
        odometry.setTarget(currentTarget[0], currentTarget[1], currentTarget[2]);
        RobotLog.d("AUTO", "Shooting target set: (" + currentTarget[0] + ", " + currentTarget[1] + ")");
    }

    private void transitionTo(AutoState newState) {
        RobotLog.d("AUTO", "Transition: " + currentState + " -> " + newState +
                " at " + String.format("%.1f", runtime.seconds()) + "s");
        currentState = newState;
        stateTimer.reset();
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Alliance", selectedAlliance.toString());
        telemetry.addLine();
        telemetry.addData("Motif", detectedMotif.toString());
        telemetry.addData("Balls Shot", ballsShot + (inSecondShootingPhase ? "/6" : "/3"));
        telemetry.addLine();
        telemetry.addData("Odometry", odometry.getTelemetryString());
        telemetry.addData("Target", goalTargeter.hasTarget() ? "YES" : "NO");
        if (goalTargeter.hasTarget()) {
            telemetry.addData("  TX", "%.1f°", goalTargeter.getTx());
        }
        telemetry.update();
    }
}
