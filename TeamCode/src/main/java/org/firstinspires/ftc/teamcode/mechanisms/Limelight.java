package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Subsystem for interacting with the Limelight 3A vision sensor.
 */
public class Limelight {

    private Limelight3A limelight;

    /**
     * Initializes the Limelight hardware.
     * 
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Poll at 100Hz
        limelight.start(); // Start processing
        limelight.pipelineSwitch(0); // Default to pipeline 0
    }

    /**
     * Gets the latest result from the Limelight.
     * 
     * @return The latest LLResult, or null if none available.
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Gets the horizontal offset from the crosshair to the target.
     * 
     * @return tx in degrees, or 0.0 if no target.
     */
    public double getTx() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0.0;
    }

    /**
     * Gets the vertical offset from the crosshair to the target.
     * 
     * @return ty in degrees, or 0.0 if no target.
     */
    public double getTy() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0.0;
    }

    /**
     * Gets the target area (0% to 100% of image).
     * 
     * @return ta, or 0.0 if no target.
     */
    public double getTa() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTa();
        }
        return 0.0;
    }

    /**
     * Switches the active pipeline.
     * 
     * @param index The index of the pipeline to switch to (0-9).
     */
    public void switchPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /**
     * Updates the robot's orientation for MegaTag 2 localization.
     * 
     * @param headingRadians The robot's heading in radians.
     */
    public void updateRobotOrientation(double headingRadians) {
        limelight.updateRobotOrientation(Math.toDegrees(headingRadians));
    }

    /**
     * Gets the robot's 3D pose from MegaTag 1.
     * 
     * @return Pose3D object, or null if invalid.
     */
    public Pose3D getBotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    /**
     * Gets the robot's 3D pose from MegaTag 2 (requires IMU update).
     * 
     * @return Pose3D object, or null if invalid.
     */
    public Pose3D getBotPoseMT2() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    /**
     * Checks if a specific AprilTag ID is currently visible.
     *
     * @param tagId The AprilTag ID to check for.
     * @return true if the specified tag is visible.
     */
    public boolean isTagVisible(int tagId) {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == tagId) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Checks if any valid target is currently visible.
     *
     * @return true if a valid target is detected.
     */
    public boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }
}
