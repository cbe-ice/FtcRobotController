package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Reusable 3-wheel dead-wheel odometry subsystem.
 * Tracks robot position (x, y) and heading on the field.
 *
 * Coordinate system:
 * - X: Forward/backward (positive = forward)
 * - Y: Left/right (positive = left)
 * - Heading: Radians, CCW positive, 0 = starting orientation
 */
public class OdometrySubsystem {

    // Hardware
    private DcMotorEx leftEncoder, rightEncoder, perpEncoder;
    private IMU imu;

    // Odometry parameters (TUNE THESE FOR YOUR ROBOT)
    private static final double TICKS_PER_REV = 8192.0; // REV Through Bore Encoder
    private static final double WHEEL_RADIUS = 1.0; // inches (for 2" omni wheel)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS);
    private static final double TRACK_WIDTH = 12.0; // inches between parallel encoders
    private static final double FORWARD_OFFSET = 6.0; // perpendicular encoder offset from center

    // Position state
    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;

    // Previous encoder values
    private int lastLeftPos = 0;
    private int lastRightPos = 0;
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
        leftEncoder = hwMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hwMap.get(DcMotorEx.class, "rightEncoder");
        perpEncoder = hwMap.get(DcMotorEx.class, "perpEncoder");

        // Set encoder directions (adjust if needed)
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        perpEncoder.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);

        reset();
    }

    /**
     * Resets position to origin and zeroes encoders.
     */
    public void reset() {
        lastLeftPos = leftEncoder.getCurrentPosition();
        lastRightPos = rightEncoder.getCurrentPosition();
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
        int currentLeftPos = leftEncoder.getCurrentPosition();
        int currentRightPos = rightEncoder.getCurrentPosition();
        int currentPerpPos = perpEncoder.getCurrentPosition();
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate deltas
        int deltaLeft = currentLeftPos - lastLeftPos;
        int deltaRight = currentRightPos - lastRightPos;
        int deltaPerp = currentPerpPos - lastPerpPos;

        // Update last positions
        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastPerpPos = currentPerpPos;

        // Convert ticks to inches
        double leftDist = deltaLeft / TICKS_PER_INCH;
        double rightDist = deltaRight / TICKS_PER_INCH;
        double perpDist = deltaPerp / TICKS_PER_INCH;

        // Calculate change in heading
        double deltaHeading = AngleUnit.normalizeRadians(currentHeading - lastHeading);

        // Calculate local movement (robot-relative)
        double localDeltaX = (leftDist + rightDist) / 2.0;
        double localDeltaY = perpDist - (FORWARD_OFFSET * deltaHeading);

        // Average heading for integration
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
