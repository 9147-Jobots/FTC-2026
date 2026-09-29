package org.firstinspires.ftc.teamcode.framework;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * A command that executes a Runnable action once in init() and finishes immediately.
 */
@SuppressWarnings("unused")
public class InstantCommand implements Command {
    private final Runnable action;
    private final Set<Subsystem> requirements;

    public InstantCommand(Runnable action, Subsystem... requirements) {
        this.action = action;
        Set<Subsystem> reqs = new HashSet<>();
        Collections.addAll(reqs, requirements);
        this.requirements = Collections.unmodifiableSet(reqs);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void init() {
        if (action != null) {
            action.run();
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
