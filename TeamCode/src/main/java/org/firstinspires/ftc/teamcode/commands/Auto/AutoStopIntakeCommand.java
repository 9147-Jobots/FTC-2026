package org.firstinspires.ftc.teamcode.commands.Auto;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for running the intake (while keeping the indexer stopped) for a specified duration (default 5 seconds) in autonomous.
 */
@SuppressWarnings("unused")
public class AutoStopIntakeCommand implements Command {
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public AutoStopIntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(intakeSubsystem);
    }

    @Override
    public void init() {
        intakeSubsystem.stopIntake(); indexerSubsystem.stopIndexer();
    }

    @Override
    public void execute() {
        // Non-blocking wait while running intake
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
