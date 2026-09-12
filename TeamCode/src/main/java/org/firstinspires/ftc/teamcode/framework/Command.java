package org.firstinspires.ftc.teamcode.framework;

/**
 * Interface representing a repeatable or state-dependent robot action routine.
 */
public interface Command {
    /** Called once when the command is scheduled. */
    void init();
    
    /** Called repeatedly while the command is active. */
    void execute();
    
    /** Returns true when the command has completed its routine. */
    boolean isFinished();
    
    /** Called when the command ends or is cancelled/interrupted. */
    void end(boolean interrupted);
}
