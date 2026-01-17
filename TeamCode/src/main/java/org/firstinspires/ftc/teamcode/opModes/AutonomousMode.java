package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * Basic Autonomous OpMode.
 * Currently just initializes the drive and waits.
 */
@Autonomous(name = "Basic Auto", group = "Auto")
public class AutonomousMode extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        drive.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            // TODO: Add autonomous logic here
            // Currently just holds position (0,0,0) for 30 seconds
            drive.drive(-1, 0, 0);
            sleep(30000);
        }
    }
}