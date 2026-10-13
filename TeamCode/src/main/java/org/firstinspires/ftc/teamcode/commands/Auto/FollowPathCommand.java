package org.firstinspires.ftc.teamcode.commands.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for executing Pedro Pathing v3.0.1 trajectories (Path).
 */
@SuppressWarnings("unused")
public class FollowPathCommand implements Command {
    private final DriveSubsystem drive;
    private final Follower follower;
    private final Path path;

    public FollowPathCommand(DriveSubsystem drive, Follower follower, Path path) {
        this.drive = drive;
        this.follower = follower;
        this.path = path;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public void init() {
        if (follower != null && path != null) {
            follower.follow(path);
        }
    }

    @Override
    public void execute() {
        if (follower != null) {
            follower.update();
        }
    }

    @Override
    public boolean isFinished() {
        return follower == null || !follower.following();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && drive != null) {
            drive.drive(0, 0, 0);
        }
    }
}
