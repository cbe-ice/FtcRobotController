package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleFlywheel {

    private DcMotor leftFlywheel, rightFlywheel;

    public void init(HardwareMap hwMap) {

        leftFlywheel = hwMap.get(DcMotor.class, "leftFlyWheel");
        rightFlywheel = hwMap.get(DcMotor.class, "rightFlyWheel");

        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void shoot(double shootSpeed) {
        leftFlywheel.setPower(shootSpeed);
        rightFlywheel.setPower(shootSpeed);

    }
}
