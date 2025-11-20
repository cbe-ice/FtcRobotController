package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DoubleFlywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class TeleOpMode extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    DoubleFlywheel shoot = new DoubleFlywheel();

    double forward, strafe, rotate;
    double speedMultiplier;
    boolean speedSwitch, intakeSwitch, loadSwitch;
    float shootSwitch;

    @Override
    public void init() {

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shoot.init(hardwareMap);

        telemetry.addData("Left Stick", " Movement");
        telemetry.addData("Left Stick Down", " Speed Switch");
        telemetry.addData("Right Stick", " Rotation");
        telemetry.addData("Right Trigger", " Shoot");
        telemetry.addData("Button X", " Intake Switch");
        telemetry.addData("Button Y", " Load Switch");


    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
        speedSwitch = gamepad1.left_stick_button;
        intakeSwitch = gamepad1.x;
        loadSwitch = gamepad1.y;
        shootSwitch = gamepad1.right_trigger;


        if(speedSwitch) {
            speedMultiplier = 0.5;
        }
        else {
            speedMultiplier = 1;
        }

        if(intakeSwitch) {
            intake.intake(0);
        }
        else {
            intake.intake(1);
        }

        if(loadSwitch) {
            intake.load(1, 75, 15);
        }

        if(shootSwitch > 0) {
            shoot.shoot(0.355, 100, 50);
        }

        drive.driveFieldRelative(forward*speedMultiplier, strafe*speedMultiplier, rotate*speedMultiplier);



    }
}