package org.firstinspires.ftc.teamcode.commands.Auto;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command responsible for mapping joystick actions dynamically to the robot drivetrain.
 */
public class AutoMove implements Command {
    private final DriveSubsystem drive;

    public AutoMove(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void init() {
        drive.drive(10, 0, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true; // Auto commands runs
    }

    @Override
    public void end(boolean interrupted) {}
}
