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
    double intakeSpeed, loadSpeed;
    double shootSpeed;
    boolean speedSwitch;
    boolean intakeSwitch, loadSwitch;
    boolean shootSwitch;

    @Override
    public void init() {

        drive.init(hardwareMap);
        intake.init(hardwareMap);
        shoot.init(hardwareMap);

        telemetry.addData("Left Stick", " Movement");
        telemetry.addData("Left Stick Down", " Speed Switch");
        telemetry.addData("Right Stick", " Rotation");
        telemetry.addData("Button A", " Shoot");
        telemetry.addData("Button X", " Intake Switch");
        telemetry.addData("Button Y", " Load Switch");


    }

    @Override
    public void loop() {

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
        speedSwitch = gamepad1.left_stick_button;
        intakeSpeed = 1;
        loadSpeed = 1;
        shootSpeed = 1;
        intakeSwitch = gamepad1.x;
        loadSwitch = gamepad1.y;
        shootSwitch = gamepad1.a;


        if(speedSwitch) {

            drive.drive(forward*0.5, strafe*0.5, rotate*0.5);
        }
        else if(!speedSwitch) {

            drive.drive(forward, strafe, rotate);
        }

        if(intakeSwitch) {

            intakeSpeed = 0;
        }
        else if(!intakeSwitch){

            intakeSpeed = 1;
        }

        if(loadSwitch) {

            loadSpeed = 1;
        }
        else if(!loadSwitch){

            loadSpeed = 0.5;
        }

        intake.intake(intakeSpeed, loadSpeed);

        if(shootSwitch) {
            shoot.shoot(shootSpeed);
        }


    }
}