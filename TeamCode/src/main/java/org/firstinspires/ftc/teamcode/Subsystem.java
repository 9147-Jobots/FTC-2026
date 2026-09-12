package org.firstinspires.ftc.teamcode;

/**
 * Common interface representing a modular robot subsystem component.
 */
public interface Subsystem {
    /**
     * Called once during the OpMode's initialization phase.
     */
    void init();

    /**
     * Called repeatedly inside the main OpMode run loop execution phase.
     */
    void periodic();
}
