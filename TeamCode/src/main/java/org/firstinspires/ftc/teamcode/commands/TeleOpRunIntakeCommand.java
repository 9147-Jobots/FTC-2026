package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Command responsible for mapping joystick actions dynamically to the robot drivetrain.
 */
public class TeleOpRunIntakeCommand implements Command {
    private final IntakeSubsystem intakeSubsystem;
    private Boolean intakeRunning;
    private final Gamepad gamepad;

    public TeleOpRunIntakeCommand(IntakeSubsystem intakeSubsystem, Gamepad gamepad) {
        this.intakeRunning = false;
        this.intakeSubsystem = intakeSubsystem;
        this.gamepad = gamepad;
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        if (!intakeRunning && gamepad.a) {
            intakeSubsystem.runIntake();
            intakeRunning = true;
        } else if (intakeRunning && !gamepad.a) {
            intakeSubsystem.stopIntake();
            intakeRunning = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false; // TeleOp commands run continuously until explicitly stopped by an operator
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        intakeRunning = false;
    }
}
