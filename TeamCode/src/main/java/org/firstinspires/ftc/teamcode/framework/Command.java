package org.firstinspires.ftc.teamcode.framework;

import java.util.Collections;
import java.util.Set;

/**
 * Interface representing a repeatable or state-dependent robot action routine.
 */
@SuppressWarnings("unused")
public interface Command {
    /** Called once when the command is scheduled. */
    void init();
    
    /** Called repeatedly while the command is active. */
    void execute();
    
    /** Returns true when the command has completed its routine. */
    boolean isFinished();
    
    /** Called when the command ends or is cancelled/interrupted. */
    void end(boolean interrupted);

    /** Returns the set of subsystems required by this command. Defaults to empty set. */
    default Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }

    /** Decorator to compose this command sequentially with another command. */
    default SequentialCommandGroup andThen(Command... next) {
        SequentialCommandGroup group = new SequentialCommandGroup(this);
        for (Command c : next) {
            group.addCommand(c);
        }
        return group;
    }

    /** Decorator to compose this command in parallel with other commands. */
    default ParallelCommandGroup alongWith(Command... parallel) {
        ParallelCommandGroup group = new ParallelCommandGroup(this);
        for (Command c : parallel) {
            group.addCommand(c);
        }
        return group;
    }
}

