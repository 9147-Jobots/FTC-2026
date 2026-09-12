package org.firstinspires.ftc.teamcode.framework;

import java.util.ArrayList;
import java.util.List;

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
        activeCommands.add(command);
        command.init();
    }

    /**
     * Executes one cycle of the robot's control loop.
     * Runs all registered subsystem periodics and manages active commands.
     */
    public void run() {
        // Run subsystem periodics automatically
        for (Subsystem subsystem : registeredSubsystems) {
            subsystem.periodic();
        }

        // Run and manage active commands lifecycle
        List<Command> toRemove = new ArrayList<>();
        for (Command command : activeCommands) {
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
