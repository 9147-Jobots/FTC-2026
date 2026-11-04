package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for controlling the shooter hood angle using position control
 * with 5-degree incremental steps on bumpers and instant max/min preset buttons on D-pad LEFT/RIGHT.
 */
@SuppressWarnings("unused")
public class TeleOpHoodCommand implements Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private static final double FIVE_DEGREES = 5.0;

    public TeleOpHoodCommand(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
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
        // Left bumper increases hood angle by 5 degrees on rising edge
        if (gamepad.left_bumper && !lastLeftBumper) {
            shooterSubsystem.adjustHoodAngleDegrees(FIVE_DEGREES);
        }
        lastLeftBumper = gamepad.left_bumper;

        // Right bumper decreases hood angle by 5 degrees on rising edge
        if (gamepad.right_bumper && !lastRightBumper) {
            shooterSubsystem.adjustHoodAngleDegrees(-FIVE_DEGREES);
        }
        lastRightBumper = gamepad.right_bumper;

        // D-pad RIGHT sets hood directly to Max Position (1.0 / 300°)
        if (gamepad.dpad_right && !lastDpadRight) {
            shooterSubsystem.setHoodMax();
        }
        lastDpadRight = gamepad.dpad_right;

        // D-pad LEFT sets hood directly to Min Position (0.0 / 0°)
        if (gamepad.dpad_left && !lastDpadLeft) {
            shooterSubsystem.setHoodMin();
        }
        lastDpadLeft = gamepad.dpad_left;
    }

    @Override
    public boolean isFinished() {
        return false; // TeleOp commands run continuously until explicitly stopped by an operator
    }

    @Override
    public void end(boolean interrupted) {
        // Hood position remains in place when command ends
    }
}
