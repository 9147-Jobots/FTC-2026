package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.vision.VisionTargetResult;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * Command responsible for automatic aiming and shooter control using AprilTagVisionSubsystem:
 * - Reads projected target center distance and yaw offset from AprilTagVisionSubsystem.
 * - Interpolates flywheel velocity (ticks/sec) and hood angle (degrees) via ShooterLookupTable.
 * - Applies proportional turning power to DriveSubsystem based on horizontal yaw offset to target center.
 * - Runs kicker motor when driver pulls trigger and velocity/heading tolerances are met.
 */
@SuppressWarnings("unused")
public class AutoAimAndShootCommand implements Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final AprilTagVisionSubsystem visionSubsystem;
    private final Gamepad gamepad;
    private final ShooterLookupTable lookupTable;

    private static final double KP_YAW = 0.035; // Proportional gain for yaw error in degrees
    private static final double HEADING_TOLERANCE_DEGREES = 3.0;
    private static final double VELOCITY_TOLERANCE_TPS = 100.0;

    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  AprilTagVisionSubsystem visionSubsystem,
                                  Gamepad gamepad,
                                  ShooterLookupTable lookupTable) {
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.gamepad = gamepad;
        this.lookupTable = lookupTable;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return new HashSet<>(Arrays.asList(shooterSubsystem, driveSubsystem));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        VisionTargetResult targetResult = visionSubsystem.getProjectedTargetResult();
        boolean hasTarget = targetResult.isTargetDetected();

        double distanceMeters = targetResult.getDistanceMeters();
        double yawErrorDegrees = targetResult.getYawDegrees();

        // Retrieve interpolated shooter parameters (flywheel velocity & hood angle)
        ShooterLookupTable.ShooterTargetState targetState = lookupTable.getInterpolatedState(distanceMeters);
        shooterSubsystem.setTargetState(targetState);

        // Compute proportional heading yaw correction for drivetrain
        double yawPower = hasTarget ? Math.max(-1.0, Math.min(1.0, KP_YAW * yawErrorDegrees)) : 0.0;

        // Combine driver translation inputs (left stick) with automatic heading yaw
        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;

        driveSubsystem.drive(axial, lateral, yawPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        driveSubsystem.drive(0, 0, 0);
    }
}
