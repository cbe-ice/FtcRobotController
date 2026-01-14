package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.mechanisms.Limelight;

/**
 * Uses the Limelight to track and target game elements.
 * Provides steering corrections and distance estimates for autonomous
 * alignment.
 */
public class GoalTargeter {

    private final Limelight limelight;

    // Tuning constants
    private static final double STEERING_KP = 0.03; // Proportional gain for steering
    private static final double DRIVE_KP = 0.02; // Proportional gain for driving
    private static final double TARGET_AREA = 5.0; // Target area percentage for ideal distance
    private static final double TX_TOLERANCE = 2.0; // Degrees of acceptable horizontal error
    private static final double TA_TOLERANCE = 0.5; // Acceptable area error percentage

    // State
    private VisionData lastVisionData = VisionData.empty();
    private boolean isLocked = false;

    /**
     * Creates a GoalTargeter with the given Limelight.
     *
     * @param limelight The initialized Limelight subsystem.
     */
    public GoalTargeter(Limelight limelight) {
        this.limelight = limelight;
    }

    /**
     * Updates the targeter with the latest vision data.
     * Call this every loop iteration.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            lastVisionData = VisionData.fromTarget(
                    result.getTx(),
                    result.getTy(),
                    result.getTa());
            isLocked = isOnTarget();
        } else {
            lastVisionData = VisionData.empty();
            isLocked = false;
        }
    }

    /**
     * Gets the steering correction needed to center on the target.
     * Positive = turn right, Negative = turn left.
     *
     * @return Steering power (-1.0 to 1.0), or 0 if no target.
     */
    public double getSteeringCorrection() {
        if (!lastVisionData.hasTarget()) {
            return 0.0;
        }

        double steer = -lastVisionData.getTx() * STEERING_KP;
        return clamp(steer, -1.0, 1.0);
    }

    /**
     * Gets the drive correction needed to reach the target distance.
     * Positive = drive forward, Negative = drive backward.
     *
     * @return Drive power (-1.0 to 1.0), or 0 if no target.
     */
    public double getDriveCorrection() {
        if (!lastVisionData.hasTarget()) {
            return 0.0;
        }

        // Area-based distance estimation
        double areaError = TARGET_AREA - lastVisionData.getTa();
        double drive = areaError * DRIVE_KP;
        return clamp(drive, -1.0, 1.0);
    }

    /**
     * Checks if the robot is centered and at the correct distance.
     *
     * @return true if on target within tolerance.
     */
    public boolean isOnTarget() {
        if (!lastVisionData.hasTarget()) {
            return false;
        }

        boolean horizontallyCentered = Math.abs(lastVisionData.getTx()) < TX_TOLERANCE;
        boolean correctDistance = Math.abs(TARGET_AREA - lastVisionData.getTa()) < TA_TOLERANCE;

        return horizontallyCentered && correctDistance;
    }

    /**
     * Checks if a target is currently visible.
     *
     * @return true if target is detected.
     */
    public boolean hasTarget() {
        return lastVisionData.hasTarget() && lastVisionData.isFresh(500);
    }

    /**
     * Checks if the targeter has achieved lock on the target.
     *
     * @return true if locked on target.
     */
    public boolean isLocked() {
        return isLocked;
    }

    /**
     * Gets the latest vision data.
     *
     * @return The most recent VisionData.
     */
    public VisionData getVisionData() {
        return lastVisionData;
    }

    /**
     * Gets the horizontal offset to target.
     *
     * @return tx in degrees, or 0 if no target.
     */
    public double getTx() {
        return lastVisionData.getTx();
    }

    /**
     * Gets the vertical offset to target.
     *
     * @return ty in degrees, or 0 if no target.
     */
    public double getTy() {
        return lastVisionData.getTy();
    }

    /**
     * Gets the target area.
     *
     * @return ta as percentage, or 0 if no target.
     */
    public double getTa() {
        return lastVisionData.getTa();
    }

    /**
     * Switches the Limelight pipeline.
     *
     * @param pipeline Pipeline index (0-9).
     */
    public void setPipeline(int pipeline) {
        limelight.switchPipeline(pipeline);
    }

    /**
     * Clamps a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Gets telemetry data as a formatted string.
     */
    public String getTelemetryString() {
        if (!lastVisionData.hasTarget()) {
            return "GoalTargeter: No Target";
        }
        return String.format("GoalTargeter: tx=%.1f° ty=%.1f° ta=%.1f%% locked=%s",
                lastVisionData.getTx(),
                lastVisionData.getTy(),
                lastVisionData.getTa(),
                isLocked ? "YES" : "NO");
    }
}