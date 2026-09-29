package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A command that pauses execution for a specified duration in seconds.
 */
public class WaitCommand implements Command {
    private final ElapsedTime timer = new ElapsedTime();
    private final double durationSeconds;

    public WaitCommand(double durationSeconds) {
        this.durationSeconds = durationSeconds;
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void execute() {
        // Non-blocking wait
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
