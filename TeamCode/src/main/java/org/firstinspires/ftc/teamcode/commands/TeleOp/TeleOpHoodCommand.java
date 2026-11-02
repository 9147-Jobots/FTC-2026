package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Collections;
import java.util.Set;

public class TeleOpHoodCommand implements Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;

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
        if (gamepad.left_bumper) {
            shooterSubsystem.setHoodPosition(shooterSubsystem.getHoodPosition()+10);
        } else if (gamepad.right_bumper) {
            shooterSubsystem.setHoodPosition(shooterSubsystem.getHoodPosition()-10);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // TeleOp commands run continuously until explicitly stopped by an operator
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}
