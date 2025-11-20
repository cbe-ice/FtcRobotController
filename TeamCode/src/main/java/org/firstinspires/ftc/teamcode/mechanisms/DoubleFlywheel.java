package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleFlywheel {

    private DcMotor leftFlywheel, rightFlywheel;
    double timer;

    public void init(HardwareMap hwMap) {

        leftFlywheel = hwMap.get(DcMotor.class, "leftFlyWheel");
        rightFlywheel = hwMap.get(DcMotor.class, "rightFlyWheel");

        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void shoot(double shootSpeed, double runtime, double cooldown) {
        if (timer <= runtime) {
            leftFlywheel.setPower(shootSpeed);
            rightFlywheel.setPower(shootSpeed);
            timer += 1;
        } else if (timer <= runtime + cooldown) {
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            timer += 1;
        } else {
            timer = 0;
        }

    }

}
