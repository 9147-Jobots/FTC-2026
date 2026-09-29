package org.firstinspires.ftc.teamcode.framework;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

/**
 * Central registry managing the execution of subsystems and scheduled commands.
 */
public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Subsystem> registeredSubsystems = new ArrayList<>();
    private final List<Command> activeCommands = new ArrayList<>();

    private CommandScheduler() {}

    public static synchronized CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    /** Registers a subsystem to have its periodic() called on every scheduler run. */
    public void registerSubsystem(Subsystem subsystem) {
        if (!registeredSubsystems.contains(subsystem)) {
            registeredSubsystems.add(subsystem);
        }
    }

    /** Schedules a new command to be initialized and tracked for execution. */
    public void schedule(Command command) {
        if (command == null || activeCommands.contains(command)) {
            return;
        }

        // Interrupt any currently scheduled commands that share subsystem requirements
        Set<Subsystem> requirements = command.getRequirements();
        if (!requirements.isEmpty()) {
            List<Command> toCancel = new ArrayList<>();
            for (Command active : activeCommands) {
                if (!Collections.disjoint(active.getRequirements(), requirements)) {
                    toCancel.add(active);
                }
            }
            for (Command active : toCancel) {
                cancel(active);
            }
        }

        activeCommands.add(command);
        command.init();
    }

    /** Cancels an active command, calling end(true). */
    public void cancel(Command command) {
        if (activeCommands.remove(command)) {
            command.end(true);
        }
    }

    /** Cancels all active commands. */
    public void cancelAll() {
        List<Command> copy = new ArrayList<>(activeCommands);
        activeCommands.clear();
        for (Command command : copy) {
            command.end(true);
        }
    }

    /** Returns true if the specified command is currently scheduled. */
    public boolean isScheduled(Command command) {
        return activeCommands.contains(command);
    }

    /**
     * Executes one cycle of the robot's control loop.
     * Runs all registered subsystem periodic and manages active commands.
     */
    public void run() {
        // Run subsystem periodic automatically
        for (Subsystem subsystem : registeredSubsystems) {
            subsystem.periodic();
        }

        // Run and manage active commands lifecycle
        List<Command> toRemove = new ArrayList<>();
        for (int i = 0; i < activeCommands.size(); i++) {
            Command command = activeCommands.get(i);
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                toRemove.add(command);
            }
        }
        activeCommands.removeAll(toRemove);
    }

    /** Clears all registrations and commands (useful for OpMode restarts). */
    public void reset() {
        registeredSubsystems.clear();
        activeCommands.clear();
    }
}

