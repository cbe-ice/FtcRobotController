package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for a double flywheel shooter mechanism.
 * Controls two motors spinning in opposite directions to launch game elements.
 */
public class DoubleFlywheel {

    private DcMotor leftFlywheel, rightFlywheel;

    /**
     * Initializes the flywheel motors.
     * 
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {

        leftFlywheel = hwMap.get(DcMotor.class, "leftFlyWheel");
        rightFlywheel = hwMap.get(DcMotor.class, "rightFlyWheel");

        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Sets the power for both flywheels.
     * 
     * @param shootSpeed Power level (0.0 to 1.0 usually)
     */
    public void shoot(double shootSpeed) {
        leftFlywheel.setPower(shootSpeed);
        rightFlywheel.setPower(shootSpeed);

    }

}