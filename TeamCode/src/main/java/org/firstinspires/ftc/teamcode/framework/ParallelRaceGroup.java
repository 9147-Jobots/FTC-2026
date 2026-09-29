package org.firstinspires.ftc.teamcode.framework;

import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Executes a group of commands simultaneously.
 * Finishes as soon as ANY child command completes, interrupting all remaining commands.
 */
public class ParallelRaceGroup implements Command {
    private final Map<Command, Boolean> commands = new LinkedHashMap<>();
    private final Set<Subsystem> requirements = new HashSet<>();
    private boolean isRunning = false;
    private boolean finishedByRace = false;

    public ParallelRaceGroup(Command... commands) {
        for (Command command : commands) {
            addCommand(command);
        }
    }

    /** Adds a command to the race group. */
    public final void addCommand(Command command) {
        if (command == null) return;
        if (isRunning) {
            throw new IllegalStateException("Cannot add commands to a ParallelRaceGroup while it is running!");
        }
        if (!Collections.disjoint(requirements, command.getRequirements())) {
            throw new IllegalArgumentException("Multiple commands in a ParallelRaceGroup cannot require the same subsystem!");
        }
        commands.put(command, false);
        requirements.addAll(command.getRequirements());
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void init() {
        isRunning = true;
        finishedByRace = false;
        for (Map.Entry<Command, Boolean> entry : commands.entrySet()) {
            entry.setValue(false);
            entry.getKey().init();
        }
    }

    @Override
    public void execute() {
        for (Map.Entry<Command, Boolean> entry : commands.entrySet()) {
            if (entry.getValue()) {
                continue;
            }

            Command command = entry.getKey();
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                entry.setValue(true);
                finishedByRace = true;
                break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finishedByRace;
    }

    @Override
    public void end(boolean interrupted) {
        for (Map.Entry<Command, Boolean> entry : commands.entrySet()) {
            if (!entry.getValue()) {
                entry.getKey().end(true);
            }
        }
        isRunning = false;
    }
}
