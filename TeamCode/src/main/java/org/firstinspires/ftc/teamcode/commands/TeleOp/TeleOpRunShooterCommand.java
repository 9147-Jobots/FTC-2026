package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

public class TeleOpRunShooterCommand implements Command {
    private final ShooterSubsystem shooterSubsystem;
    private Boolean shooterRunning;
    private final Gamepad gamepad;

    public TeleOpRunShooterCommand(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this.shooterRunning = false;
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
        if (!shooterRunning && gamepad.right_trigger_pressed) {
            shooterSubsystem.runShooterPower(1);
            shooterRunning = true;
        } else if (shooterRunning && !gamepad.right_trigger_pressed) {
            shooterSubsystem.stopShooter();
            shooterRunning = false;
        }

        if (gamepad.left_bumper) {
            shooterSubsystem.setHoodPosition(shooterSubsystem.getHoodPosition()+10);
        } else if (gamepad.right_bumper) {
            shooterSubsystem.setHoodPosition(shooterSubsystem.getHoodPosition()-10);
        }

//        if (gamepad.b) {
//            intakeSubsystem.runServos();
//        } else {
//            intakeSubsystem.stopServos();
//        }
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
