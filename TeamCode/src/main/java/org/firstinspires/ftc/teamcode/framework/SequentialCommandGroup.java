package org.firstinspires.ftc.teamcode.framework;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Executes a sequence of commands in order.
 * Advances to the next command when the current command finishes.
 */
public class SequentialCommandGroup implements Command {
    private final List<Command> commands = new ArrayList<>();
    private final Set<Subsystem> requirements = new HashSet<>();
    private int currentCommandIndex = -1;

    public SequentialCommandGroup(Command... commands) {
        for (Command command : commands) {
            addCommand(command);
        }
    }

    /** Adds a command to the end of the sequence. */
    public final void addCommand(Command command) {
        if (command == null) return;
        if (currentCommandIndex != -1) {
            throw new IllegalStateException("Cannot add commands to a SequentialCommandGroup while it is running!");
        }
        commands.add(command);
        requirements.addAll(command.getRequirements());
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void init() {
        currentCommandIndex = 0;
        if (!commands.isEmpty()) {
            commands.get(0).init();
        }
    }

    @Override
    public void execute() {
        if (commands.isEmpty() || currentCommandIndex < 0 || currentCommandIndex >= commands.size()) {
            return;
        }

        Command currentCommand = commands.get(currentCommandIndex);
        currentCommand.execute();

        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            currentCommandIndex++;
            if (currentCommandIndex < commands.size()) {
                commands.get(currentCommandIndex).init();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return commands.isEmpty() || currentCommandIndex < 0 || currentCommandIndex >= commands.size();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && currentCommandIndex >= 0 && currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).end(true);
        }
        currentCommandIndex = -1;
    }
}
