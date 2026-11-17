package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for running only the shooter flywheel at a target RPM (default 1500 RPM).
 */
@SuppressWarnings("unused")
public class RunShooterRPMCommand implements Command {
    public static final double DEFAULT_TARGET_RPM = 1500.0;

    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;
    private final double targetRpm;
    private boolean shooterRunning = false;

    public RunShooterRPMCommand(ShooterSubsystem shooterSubsystem, double targetRpm) {
        this(shooterSubsystem, null, targetRpm);
    }

    public RunShooterRPMCommand(ShooterSubsystem shooterSubsystem) {
        this(shooterSubsystem, null, DEFAULT_TARGET_RPM);
    }

    public RunShooterRPMCommand(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this(shooterSubsystem, gamepad, DEFAULT_TARGET_RPM);
    }

    public RunShooterRPMCommand(ShooterSubsystem shooterSubsystem, Gamepad gamepad, double targetRpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.gamepad = gamepad;
        this.targetRpm = targetRpm;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(shooterSubsystem);
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        if (gamepad != null) {
            if (gamepad.a && !shooterRunning) {
                shooterSubsystem.runFlywheelVelocity(targetRpm);
                shooterRunning = true;
            } else if (!gamepad.a && shooterRunning) {
                shooterSubsystem.stopShooter();
                shooterRunning = false;
            }
        } else {
            shooterSubsystem.runFlywheelVelocity(targetRpm);
            shooterRunning = true;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterRunning = false;
    }
}
