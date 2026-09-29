package org.firstinspires.ftc.teamcode.framework;

/**
 * Common interface representing a modular robot subsystem component.
 */
@SuppressWarnings("unused")
public interface Subsystem {
    /**
     * Called repeatedly inside the main OpMode run loop execution phase.
     */
    void periodic();

    /** Sets the default command for this subsystem. */
    default void setDefaultCommand(Command defaultCommand) {
        CommandScheduler.getInstance().setDefaultCommand(this, defaultCommand);
    }

    /** Gets the default command for this subsystem. */
    default Command getDefaultCommand() {
        return CommandScheduler.getInstance().getDefaultCommand(this);
    }
}
