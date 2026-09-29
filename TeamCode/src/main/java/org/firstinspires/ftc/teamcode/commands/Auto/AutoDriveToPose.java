package org.firstinspires.ftc.teamcode.commands.Auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for driving the robot to a target (X, Y) coordinate and heading angle.
 */
@SuppressWarnings("unused")
public class AutoDriveToPose implements Command {
    private final DriveSubsystem drive;
    private final Odometry odometry;

    private final double targetX;
    private final double targetY;
    private final double targetHeadingDeg;

    private final double posToleranceMeters;
    private final double headingToleranceDeg;

    private double kPPos = 1.2;
    private double kPHeading = 0.02;
    private double maxPower = 0.8;

    public AutoDriveToPose(DriveSubsystem drive, Odometry odometry, double targetX, double targetY, double targetHeadingDeg) {
        this(drive, odometry, targetX, targetY, targetHeadingDeg, 0.03, 2.0);
    }

    public AutoDriveToPose(DriveSubsystem drive, Odometry odometry, double targetX, double targetY, double targetHeadingDeg, double posToleranceMeters, double headingToleranceDeg) {
        this.drive = drive;
        this.odometry = odometry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeadingDeg = targetHeadingDeg;
        this.posToleranceMeters = posToleranceMeters;
        this.headingToleranceDeg = headingToleranceDeg;
    }

    public AutoDriveToPose setGains(double kPPos, double kPHeading, double maxPower) {
        this.kPPos = kPPos;
        this.kPHeading = kPHeading;
        this.maxPower = maxPower;
        return this;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(drive);
    }

    @Override
    public void init() {
    }

    @Override
    public void execute() {
        if (odometry == null) return;

        Pose2D currentPose = odometry.getPose();
        if (currentPose == null) return;

        double currentX = currentPose.getX(DistanceUnit.METER);
        double currentY = currentPose.getY(DistanceUnit.METER);
        double currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = normalizeAngleDeg(targetHeadingDeg - currentHeadingDeg);

        // Proportional control
        double axialPower = yError * kPPos;
        double lateralPower = xError * kPPos;
        double yawPower = headingError * kPHeading;

        // Clamp power outputs
        axialPower = clamp(axialPower, -maxPower, maxPower);
        lateralPower = clamp(lateralPower, -maxPower, maxPower);
        yawPower = clamp(yawPower, -maxPower, maxPower);

        drive.drive(axialPower, lateralPower, yawPower);
    }

    @Override
    public boolean isFinished() {
        if (odometry == null) return true;

        Pose2D currentPose = odometry.getPose();
        if (currentPose == null) return true;

        double currentX = currentPose.getX(DistanceUnit.METER);
        double currentY = currentPose.getY(DistanceUnit.METER);
        double currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

        double distError = Math.hypot(targetX - currentX, targetY - currentY);
        double headingError = Math.abs(normalizeAngleDeg(targetHeadingDeg - currentHeadingDeg));

        return distError <= posToleranceMeters && headingError <= headingToleranceDeg;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double normalizeAngleDeg(double angle) {
        double result = angle % 360.0;
        if (result > 180.0) {
            result -= 360.0;
        } else if (result < -180.0) {
            result += 360.0;
        }
        return result;
    }
}
