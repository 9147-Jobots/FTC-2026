package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Command responsible for running Intake and Indexer together when Left Trigger is held.
 */
@SuppressWarnings("unused")
public class TeleOpRunIntakeCommand implements Command {
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final Gamepad gamepad;
    private boolean intakeRunning = false;

    public TeleOpRunIntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, Gamepad gamepad) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.gamepad = gamepad;
    }

    public TeleOpRunIntakeCommand(IntakeSubsystem intakeSubsystem, Gamepad gamepad) {
        this(intakeSubsystem, null, gamepad);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        if (indexerSubsystem != null) {
            return new HashSet<>(Arrays.asList(intakeSubsystem, indexerSubsystem));
        }
        return Collections.singleton(intakeSubsystem);
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        boolean leftTriggerPressed = gamepad.left_trigger > 0.2;

        if (leftTriggerPressed && !intakeRunning) {
            intakeSubsystem.runIntake();
            if (indexerSubsystem != null) {
                indexerSubsystem.runIndexer(1.0);
            }
            intakeRunning = true;
        } else if (!leftTriggerPressed && intakeRunning) {
            intakeSubsystem.stopIntake();
            if (indexerSubsystem != null) {
                indexerSubsystem.stopIndexer();
            }
            intakeRunning = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        if (indexerSubsystem != null) {
            indexerSubsystem.stopIndexer();
        }
        intakeRunning = false;
    }
}
