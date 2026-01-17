package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Subsystem for the robot's intake mechanism.
 * Controls the intake motor and the loading motor.
 */
public class Intake {

    private DcMotor intakeMotor, loadMotor;
    private ElapsedTime loadTimer = new ElapsedTime();
    private boolean timerRunning = false;

    /**
     * Initializes the intake hardware.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        loadMotor = hwMap.get(DcMotor.class, "loadMotor");
        loadTimer.reset();
    }

    /**
     * Sets the power of the intake motor.
     *
     * @param intakeSpeed Power level (-1.0 to 1.0)
     */
    public void intake(double intakeSpeed) {
        intakeMotor.setPower(intakeSpeed);
    }

    /**
     * Controls the loading mechanism with a pulsed action.
     * Uses real-time based timing for consistent behavior regardless of loop
     * frequency.
     *
     * @param loadSpeed  Power level for the loader
     * @param intervalMs Milliseconds to run the loader
     * @param cooldownMs Milliseconds to pause the loader
     */
    public void load(double loadSpeed, double intervalMs, double cooldownMs) {
        // Start timer on first call
        if (!timerRunning) {
            loadTimer.reset();
            timerRunning = true;
        }

        double elapsedMs = loadTimer.milliseconds();

        if (elapsedMs < intervalMs) {
            // Active phase: run loader
            loadMotor.setPower(loadSpeed);
        } else if (elapsedMs < intervalMs + cooldownMs) {
            // Cooldown phase: stop loader
            loadMotor.setPower(0);
        } else {
            // Cycle complete: reset timer
            loadTimer.reset();
        }
    }

    /**
     * Stops all intake motors immediately.
     * Resets the loader timer state.
     */
    public void stopAll() {
        intakeMotor.setPower(0);
        loadMotor.setPower(0);
        timerRunning = false;
        loadTimer.reset();
    }
}