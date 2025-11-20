package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Odometry Data Collector", group = "Odometry")
public class Odometry extends LinearOpMode {

    // Hardware
    private DcMotorEx leftEncoder, rightEncoder, perpEncoder;
    private IMU imu;

    // Odometry parameters (TUNE THESE FOR YOUR ROBOT)
    private static final double TICKS_PER_REV = 8192.0; // REV Through Bore Encoder
    private static final double WHEEL_RADIUS = 1.0; // inches (for 2" omni wheel)
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS);
    private static final double TRACK_WIDTH = 12.0; // inches between parallel encoders
    private static final double FORWARD_OFFSET = 6.0; // inches perpendicular encoder from center

    // Position tracking
    private double x = 0.0, y = 0.0, heading = 0.0; // inches and radians
    private int lastLeftPos = 0, lastRightPos = 0, lastPerpPos = 0;

    // Data storage
    private ArrayList<OdometryData> dataPoints = new ArrayList<>();
    private long startTime;

    // Data class
    static class OdometryData {
        long timestamp;
        double x, y, heading;
        int leftTicks, rightTicks, perpTicks;

        OdometryData(long time, double x, double y, double heading,
                     int left, int right, int perp) {
            this.timestamp = time;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.leftTicks = left;
            this.rightTicks = right;
            this.perpTicks = perp;
        }

        String toCSV() {
            return String.format("%d,%.3f,%.3f,%.3f,%d,%d,%d",
                    timestamp, x, y, Math.toDegrees(heading),
                    leftTicks, rightTicks, perpTicks);
        }
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        telemetry.addData("Status", "Initialized - Ready to collect data");
        telemetry.addData("Instructions", "Drive around to collect odometry data");
        telemetry.addData("", "Data will be saved when you stop");
        telemetry.update();

        waitForStart();
        startTime = System.currentTimeMillis();

        // Reset encoders
        resetEncoders();

        while (opModeIsActive()) {
            // Update odometry
            updateOdometry();

            // Store data point every loop
            long currentTime = System.currentTimeMillis() - startTime;
            dataPoints.add(new OdometryData(
                    currentTime, x, y, heading,
                    leftEncoder.getCurrentPosition(),
                    rightEncoder.getCurrentPosition(),
                    perpEncoder.getCurrentPosition()
            ));

            // Display real-time data
            displayTelemetry();

            // Optional: Add a small delay to control data collection rate
            // sleep(50); // Collect at ~20Hz instead of maximum rate
        }

        // Save data when OpMode stops
        saveDataToFile();
    }

    private void initHardware() {
        // Configure encoders (adjust names to match your configuration)
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        perpEncoder = hardwareMap.get(DcMotorEx.class, "perpEncoder");

        // Set encoder directions (adjust if needed)
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        perpEncoder.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
    }

    private void resetEncoders() {
        lastLeftPos = leftEncoder.getCurrentPosition();
        lastRightPos = rightEncoder.getCurrentPosition();
        lastPerpPos = perpEncoder.getCurrentPosition();

        // Reset IMU heading
        imu.resetYaw();

        // Reset position
        x = 0.0;
        y = 0.0;
        heading = 0.0;
    }

    private void updateOdometry() {
        // Read current encoder positions
        int currentLeftPos = leftEncoder.getCurrentPosition();
        int currentRightPos = rightEncoder.getCurrentPosition();
        int currentPerpPos = perpEncoder.getCurrentPosition();

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

        // Calculate change in heading (using encoders)
        double deltaHeading = (rightDist - leftDist) / TRACK_WIDTH;

        // Or use IMU for heading (more accurate)
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate local movement
        double localDeltaX = (leftDist + rightDist) / 2.0;
        double localDeltaY = perpDist - (FORWARD_OFFSET * deltaHeading);

        // Convert to global coordinates
        double deltaXGlobal = localDeltaX * Math.cos(heading) - localDeltaY * Math.sin(heading);
        double deltaYGlobal = localDeltaX * Math.sin(heading) + localDeltaY * Math.cos(heading);

        // Update global position
        x += deltaXGlobal;
        y += deltaYGlobal;
    }

    private void displayTelemetry() {
        telemetry.addData("Data Points", dataPoints.size());
        telemetry.addData("Time (s)", "%.1f", (System.currentTimeMillis() - startTime) / 1000.0);
        telemetry.addLine();
        telemetry.addData("X Position", "%.2f inches", x);
        telemetry.addData("Y Position", "%.2f inches", y);
        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(heading));
        telemetry.addLine();
        telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("Perp Encoder", perpEncoder.getCurrentPosition());
        telemetry.update();
    }

    private void saveDataToFile() {
        try {
            // Save to external storage (visible via USB)
            String filename = "/sdcard/FIRST/odometry_data_" + startTime + ".csv";
            FileWriter writer = new FileWriter(filename);

            // Write header
            writer.write("timestamp_ms,x_inches,y_inches,heading_degrees,left_ticks,right_ticks,perp_ticks\n");

            // Write data points
            for (OdometryData data : dataPoints) {
                writer.write(data.toCSV() + "\n");
            }

            writer.close();

            telemetry.addData("Success", "Data saved to:");
            telemetry.addData("", filename);
            telemetry.addData("Points", dataPoints.size());
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to save data");
            telemetry.addData("Exception", e.getMessage());
        }
        telemetry.update();
        sleep(5000); // Display message for 5 seconds
    }
}