package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for controlling the shooter flywheel and hood in TeleOp.
 * D-pad UP / DOWN increments/decrements open-loop target power (setPower).
 */
@SuppressWarnings("unused")
public class TeleOpRunShooterCommand implements Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;

    private boolean shooterRunning = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private double targetPower = 1.0;
    private static final double POWER_STEP = 0.05;

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

        // Trigger check to run flywheel at target power (open-loop setPower)
        boolean triggerPressed = gamepad.right_trigger > 0.2;
        if (triggerPressed) {
            shooterSubsystem.runShooterPower(targetPower);
            shooterRunning = true;
        } else if (shooterRunning) {
            shooterSubsystem.stopShooter();
            shooterRunning = false;
        }

        // Hood position adjustment via left/right bumpers
        if (gamepad.left_bumper) {
            shooterSubsystem.setHoodPosition(shooterSubsystem.getHoodPosition() + 0.01);
        } else if (gamepad.right_bumper) {
            shooterSubsystem.setHoodPosition(shooterSubsystem.getHoodPosition() - 0.01);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // TeleOp commands run continuously until explicitly stopped by an operator
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterRunning = false;
    }
}
