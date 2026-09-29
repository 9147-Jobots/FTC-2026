package org.firstinspires.ftc.teamcode.commands.Auto;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for driving the robot forward a specified distance in meters.
 */
@SuppressWarnings("unused")
public class AutoMove implements Command {
    private final DriveSubsystem drive;
    private final Odometry odometry;
    private final double targetDistanceMeters;
    private final double drivePower;

    private double startX = 0;
    private double startY = 0;

    public AutoMove(DriveSubsystem drive, Odometry odometry, double targetDistanceMeters, double drivePower) {
        this.drive = drive;
        this.odometry = odometry;
        this.targetDistanceMeters = targetDistanceMeters;
        this.drivePower = drivePower;
    }

    public AutoMove(DriveSubsystem drive, Odometry odometry, double targetDistanceMeters) {
        this(drive, odometry, targetDistanceMeters, 0.5);
    }

    public AutoMove(DriveSubsystem drive, Odometry odometry) {
        this(drive, odometry, 1.0, 0.5);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public void init() {
        if (odometry != null) {
            Pose2D pose = odometry.getPose();
            if (pose != null) {
                startX = pose.getX(DistanceUnit.METER);
                startY = pose.getY(DistanceUnit.METER);
            }
        }
    }

    @Override
    public void execute() {
        drive.drive(drivePower, 0, 0);
    }

    @Override
    public boolean isFinished() {
        if (odometry == null) {
            return true;
        }
        Pose2D pose = odometry.getPose();
        if (pose == null) {
            return true;
        }
        double currentX = pose.getX(DistanceUnit.METER);
        double currentY = pose.getY(DistanceUnit.METER);
        double distanceTraveled = Math.hypot(currentX - startX, currentY - startY);

        return distanceTraveled >= targetDistanceMeters;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}
