package org.firstinspires.ftc.teamcode.framework;

/**
 * Abstract base class that automatically handles scheduler registration upon instantiation.
 */
public abstract class SubsystemBase implements Subsystem {
    public SubsystemBase() {
        // Automatically registers the subclass instance with the scheduler context!
        CommandScheduler.getInstance().registerSubsystem(this);
    }
}
