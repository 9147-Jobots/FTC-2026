package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for controlling the shooter flywheel, kicker, and hood in TeleOp.
 * - Right Trigger: Flywheel spin at open-loop target power.
 * - Left Trigger: Kicker motor runs at 100% (1.0) while held, 0 on release.
 * - D-pad UP / DOWN: Increments/decrements flywheel target power.
 * - Left / Right Bumpers: Adjust hood angle by 5-degree increments.
 * - Y / X Buttons: Set hood directly to Max Position (1.0 / 300°) and Min Position (0.0 / 0°).
 */
@SuppressWarnings("unused")
public class TeleOpRunShooterCommand implements Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;

    private boolean shooterRunning = false;
    private boolean kickerRunning = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastY = false;
    private boolean lastX = false;

    private double targetPower = 1.0;
    private static final double FIVE_DEGREES = 5.0;
    private static final double POWER_STEP = 0.02;

    public TeleOpRunShooterCommand(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this.shooterSubsystem = shooterSubsystem;
        this.gamepad = gamepad;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(shooterSubsystem);
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        // D-pad UP / DOWN edge detection to increment / decrement open-loop target power
        if (gamepad.dpad_up && !lastDpadUp) {
            targetPower = Math.min(1.0, targetPower + POWER_STEP);
        }
        lastDpadUp = gamepad.dpad_up;

        if (gamepad.dpad_down && !lastDpadDown) {
            targetPower = Math.max(0.0, targetPower - POWER_STEP);
        }
        lastDpadDown = gamepad.dpad_down;

        // Right Trigger check to run flywheel at target power
        boolean rightTriggerPressed = gamepad.right_trigger > 0.2;
        if (rightTriggerPressed) {
            shooterSubsystem.runShooterPower(targetPower);
            shooterRunning = true;
        } else if (shooterRunning) {
            shooterSubsystem.stopShooter();
            shooterRunning = false;
        }

        // Left Trigger check to run kicker at 100% while held, 0 on release
        boolean leftTriggerPressed = gamepad.left_trigger > 0.2;
        if (leftTriggerPressed) {
            shooterSubsystem.runKickerPower(1.0);
            kickerRunning = true;
        } else if (kickerRunning) {
            shooterSubsystem.stopKicker();
            kickerRunning = false;
        }

        // Hood position adjustment via left/right bumpers in 5-degree jumps on rising edge
        if (gamepad.left_bumper && !lastLeftBumper) {
            shooterSubsystem.adjustHoodAngleDegrees(FIVE_DEGREES);
        }
        lastLeftBumper = gamepad.left_bumper;

        if (gamepad.right_bumper && !lastRightBumper) {
            shooterSubsystem.adjustHoodAngleDegrees(-FIVE_DEGREES);
        }
        lastRightBumper = gamepad.right_bumper;

        // Button Y sets hood directly to Max Position (1.0 / 300°)
        if (gamepad.y && !lastY) {
            shooterSubsystem.setHoodMax();
        }
        lastY = gamepad.y;

        // Button X sets hood directly to Min Position (0.0 / 0°)
        if (gamepad.x && !lastX) {
            shooterSubsystem.setHoodMin();
        }
        lastX = gamepad.x;
    }

    @Override
    public boolean isFinished() {
        return false; // TeleOp commands run continuously until explicitly stopped by an operator
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopKicker();
        shooterRunning = false;
        kickerRunning = false;
    }
}
