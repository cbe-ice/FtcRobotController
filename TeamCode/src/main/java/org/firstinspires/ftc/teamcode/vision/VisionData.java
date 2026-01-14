package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Data transfer object for vision processing results.
 * Holds target data from the Limelight or other vision sensors.
 */
public class VisionData {

    // Target offset data
    private final double tx; // Horizontal offset from crosshair to target (degrees)
    private final double ty; // Vertical offset from crosshair to target (degrees)
    private final double ta; // Target area (0% to 100% of image)

    // AprilTag data
    private final int tagID;
    private final Pose3D botPose;

    // Timestamp for data freshness
    private final long timestamp;

    // Whether this data represents a valid target
    private final boolean hasTarget;

    /**
     * Creates a VisionData object with target data.
     */
    public VisionData(double tx, double ty, double ta, int tagID, Pose3D botPose, boolean hasTarget) {
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
        this.tagID = tagID;
        this.botPose = botPose;
        this.hasTarget = hasTarget;
        this.timestamp = System.currentTimeMillis();
    }

    /**
     * Creates an empty VisionData object (no target detected).
     */
    public static VisionData empty() {
        return new VisionData(0.0, 0.0, 0.0, -1, null, false);
    }

    /**
     * Creates a VisionData object with basic target data (no AprilTag).
     */
    public static VisionData fromTarget(double tx, double ty, double ta) {
        return new VisionData(tx, ty, ta, -1, null, true);
    }

    /**
     * Creates a VisionData object with AprilTag data.
     */
    public static VisionData fromAprilTag(double tx, double ty, double ta, int tagID, Pose3D botPose) {
        return new VisionData(tx, ty, ta, tagID, botPose, true);
    }

    // --- Getters ---

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    public int getTagID() {
        return tagID;
    }

    public Pose3D getBotPose() {
        return botPose;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Checks if the data is recent (within specified milliseconds).
     */
    public boolean isFresh(long maxAgeMs) {
        return (System.currentTimeMillis() - timestamp) < maxAgeMs;
    }

    /**
     * Gets the age of this data in milliseconds.
     */
    public long getAgeMs() {
        return System.currentTimeMillis() - timestamp;
    }

    @Override
    public String toString() {
        if (!hasTarget) {
            return "VisionData[No Target]";
        }
        return String.format("VisionData[tx=%.2f, ty=%.2f, ta=%.2f, tagID=%d, age=%dms]",
                tx, ty, ta, tagID, getAgeMs());
    }
}