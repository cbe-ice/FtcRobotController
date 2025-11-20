package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor, loadMotor;
    double timer;

    public void init(HardwareMap hwMap) {

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        loadMotor = hwMap.get(DcMotor.class, "loadMotor");
    }

    public void intake(double intakeSpeed) {
        intakeMotor.setPower(intakeSpeed);
    }

    public void load(double loadSpeed, double interval, double cooldown) {

        if (timer <= interval) {
            loadMotor.setPower(loadSpeed);
            timer += 1;
        } else if (timer <= interval + cooldown) {
            loadMotor.setPower(0);
            timer += 1;
        } else {
            timer = 0;
        }
    }
}
