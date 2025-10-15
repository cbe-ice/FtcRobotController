package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous
public class AutonomousMode extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

        waitForStart();

         if (opModeIsActive()) {

            drive.driveFieldRelative(0, 0, 0);
            sleep(30000);



        }
    }
}
