package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for controlling the indexer mechanism in TeleOp (running when Gamepad B is held).
 */
@SuppressWarnings("unused")
public class TeleOpRunIndexerCommand implements Command {
    private final IndexerSubsystem indexerSubsystem;
    private final Gamepad gamepad;
    private boolean indexerRunning = false;

    public TeleOpRunIndexerCommand(IndexerSubsystem indexerSubsystem, Gamepad gamepad) {
        this.indexerSubsystem = indexerSubsystem;
        this.gamepad = gamepad;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(indexerSubsystem);
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        if (!indexerRunning && gamepad.b) {
            indexerSubsystem.runIndexer();
            indexerRunning = true;
        } else if (indexerRunning && !gamepad.b) {
            indexerSubsystem.stopIndexer();
            indexerRunning = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        indexerRunning = false;
    }
}
