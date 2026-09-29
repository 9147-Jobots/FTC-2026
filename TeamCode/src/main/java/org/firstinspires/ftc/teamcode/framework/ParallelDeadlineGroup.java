package org.firstinspires.ftc.teamcode.framework;

import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Executes a group of commands simultaneously.
 * Finishes as soon as a designated "deadline" command completes, interrupting all remaining commands.
 */
@SuppressWarnings("unused")
public class ParallelDeadlineGroup implements Command {
    private final Command deadline;
    private final Map<Command, Boolean> commands = new LinkedHashMap<>();
    private final Set<Subsystem> requirements = new HashSet<>();
    private boolean isRunning = false;

    public ParallelDeadlineGroup(Command deadline, Command... otherCommands) {
        this.deadline = deadline;
        addCommand(deadline);
        for (Command command : otherCommands) {
            addCommand(command);
        }
    }

    /** Adds a command to be run alongside the deadline command. */
    public final void addCommand(Command command) {
        if (command == null) return;
        if (isRunning) {
            throw new IllegalStateException("Cannot add commands to a ParallelDeadlineGroup while it is running!");
        }
        if (!Collections.disjoint(requirements, command.getRequirements())) {
            throw new IllegalArgumentException("Multiple commands in a ParallelDeadlineGroup cannot require the same subsystem!");
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
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Finishes as soon as the designated deadline command finishes
        Boolean deadlineFinished = commands.get(deadline);
        return deadlineFinished != null && deadlineFinished;
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
