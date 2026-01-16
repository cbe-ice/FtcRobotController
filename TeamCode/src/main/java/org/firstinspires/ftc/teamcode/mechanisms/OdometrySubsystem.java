package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * 2-wheel dead-wheel odometry subsystem with IMU heading.
 * Tracks robot position (x, y) and heading on the field.
 * 
 * Configuration:
 * - Forward encoder: measures forward/backward motion (4.75" from center)
 * - Perpendicular encoder: measures strafe/lateral motion (6.5" from center)
 * - IMU: provides heading
 *
 * Coordinate system:
 * - X: Forward/backward (positive = forward)
 * - Y: Left/right (positive = left)
 * - Heading: Radians, CCW positive, 0 = starting orientation
 */
public class OdometrySubsystem {

    // Hardware
    private DcMotorEx forwardEncoder, perpEncoder;
    private IMU imu;

    // Odometry parameters (TUNE THESE FOR YOUR ROBOT)
    private static final double TICKS_PER_REV = 8192.0; // REV Through Bore Encoder
    private static final double WHEEL_RADIUS = 1.88976378; // inches (48mm wheel)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS);

    // Encoder offsets from robot center (for rotation compensation)
    private static final double FORWARD_ENCODER_OFFSET = 4.75; // forward encoder distance from center
    private static final double PERP_ENCODER_OFFSET = 6.5; // perpendicular encoder distance from center

    // Position state
    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;

    // Previous encoder values
    private int lastForwardPos = 0;
    private int lastPerpPos = 0;
    private double lastHeading = 0.0;

    // Target position for autonomous navigation
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetHeading = 0.0;

    /**
     * Initializes the odometry hardware.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        // Configure encoders (adjust names to match your configuration)
        forwardEncoder = hwMap.get(DcMotorEx.class, "forwardEncoder");
        perpEncoder = hwMap.get(DcMotorEx.class, "perpEncoder");

        // Set encoder directions (adjust if needed)
        forwardEncoder.setDirection(DcMotor.Direction.FORWARD);
        perpEncoder.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU - orientation must match MecanumDrive for consistency
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(imuParams);

        reset();
    }

    /**
     * Resets position to origin and zeroes encoders.
     */
    public void reset() {
        lastForwardPos = forwardEncoder.getCurrentPosition();
        lastPerpPos = perpEncoder.getCurrentPosition();

        imu.resetYaw();

        x = 0.0;
        y = 0.0;
        heading = 0.0;
        lastHeading = 0.0;
    }

    /**
     * Sets the current position (for field-relative starting positions).
     */
    public void setPosition(double x, double y, double headingRadians) {
        this.x = x;
        this.y = y;
        this.heading = headingRadians;
        this.lastHeading = headingRadians;
    }

    /**
     * Updates the odometry calculation.
     * Call this every loop iteration.
     */
    public void update() {
        // Read current encoder positions
        int currentForwardPos = forwardEncoder.getCurrentPosition();
        int currentPerpPos = perpEncoder.getCurrentPosition();
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate deltas
        int deltaForward = currentForwardPos - lastForwardPos;
        int deltaPerp = currentPerpPos - lastPerpPos;

        // Update last positions
        lastForwardPos = currentForwardPos;
        lastPerpPos = currentPerpPos;

        // Convert ticks to inches
        double forwardDist = deltaForward / TICKS_PER_INCH;
        double perpDist = deltaPerp / TICKS_PER_INCH;

        // Calculate change in heading from IMU
        double deltaHeading = AngleUnit.normalizeRadians(currentHeading - lastHeading);

        // Calculate local movement (robot-relative)
        // Compensate for encoder arc during rotation
        double localDeltaX = forwardDist - (FORWARD_ENCODER_OFFSET * deltaHeading);
        double localDeltaY = perpDist - (PERP_ENCODER_OFFSET * deltaHeading);

        // Average heading for integration (more accurate during turns)
        double avgHeading = lastHeading + (deltaHeading / 2.0);

        // Convert to global coordinates (field-relative)
        double deltaXGlobal = localDeltaX * Math.cos(avgHeading) - localDeltaY * Math.sin(avgHeading);
        double deltaYGlobal = localDeltaX * Math.sin(avgHeading) + localDeltaY * Math.cos(avgHeading);

        // Update global position
        x += deltaXGlobal;
        y += deltaYGlobal;
        heading = currentHeading;
        lastHeading = currentHeading;
    }

    // --- Position Getters ---

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    // --- Target Management ---

    /**
     * Sets a target position for autonomous navigation.
     */
    public void setTarget(double targetX, double targetY, double targetHeadingRadians) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = targetHeadingRadians;
    }

    /**
     * Gets the distance to the current target.
     */
    public double getDistanceToTarget() {
        double dx = targetX - x;
        double dy = targetY - y;
        return Math.hypot(dx, dy);
    }

    /**
     * Gets the angle to the target (field-relative).
     */
    public double getAngleToTarget() {
        double dx = targetX - x;
        double dy = targetY - y;
        return Math.atan2(dy, dx);
    }

    /**
     * Gets the heading error to target heading.
     */
    public double getHeadingError() {
        return AngleUnit.normalizeRadians(targetHeading - heading);
    }

    /**
     * Checks if we've reached the target position.
     *
     * @param toleranceInches  Position tolerance in inches.
     * @param toleranceRadians Heading tolerance in radians.
     */
    public boolean isAtTarget(double toleranceInches, double toleranceRadians) {
        return getDistanceToTarget() < toleranceInches &&
                Math.abs(getHeadingError()) < toleranceRadians;
    }

    // --- Drive Corrections for Autonomous ---

    /**
     * Calculates drive power to reach target (proportional control).
     *
     * @param kP       Proportional gain.
     * @param maxPower Maximum power output.
     * @return Forward power (-1 to 1).
     */
    public double getDriveCorrection(double kP, double maxPower) {
        double distance = getDistanceToTarget();
        double power = distance * kP;
        return clamp(power, -maxPower, maxPower);
    }

    /**
     * Calculates strafe power to reach target (proportional control).
     *
     * @param kP       Proportional gain.
     * @param maxPower Maximum power output.
     * @return Strafe power (-1 to 1).
     */
    public double getStrafeCorrection(double kP, double maxPower) {
        double angleToTarget = getAngleToTarget();
        double relativeAngle = AngleUnit.normalizeRadians(angleToTarget - heading);

        // Strafe component based on angle difference
        double power = Math.sin(relativeAngle) * getDistanceToTarget() * kP;
        return clamp(power, -maxPower, maxPower);
    }

    /**
     * Calculates rotation power to face target heading.
     *
     * @param kP       Proportional gain.
     * @param maxPower Maximum power output.
     * @return Rotation power (-1 to 1).
     */
    public double getRotationCorrection(double kP, double maxPower) {
        double error = getHeadingError();
        double power = error * kP;
        return clamp(power, -maxPower, maxPower);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Returns telemetry string for debugging.
     */
    public String getTelemetryString() {
        return String.format("Pos: (%.1f, %.1f) in | Heading: %.1f° | Dist to target: %.1f in",
                x, y, Math.toDegrees(heading), getDistanceToTarget());
    }
}
