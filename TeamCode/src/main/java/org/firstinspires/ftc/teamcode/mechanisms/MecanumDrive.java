package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrive {
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private IMU imu;

    public void init(HardwareMap hwMap) {

        leftFrontMotor = hwMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hwMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hwMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hwMap.get(DcMotor.class, "rightBackMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {

        double leftFrontPower = forward + strafe + rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightBackPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontMotor.setPower(maxSpeed * (leftFrontPower / maxPower));
        leftBackMotor.setPower(maxSpeed * (leftBackPower / maxPower));
        rightFrontMotor.setPower(maxSpeed * (rightFrontPower / maxPower));
        rightBackMotor.setPower(maxSpeed * (rightBackPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);
        this.drive(newForward, newStrafe, rotate);
    }
}