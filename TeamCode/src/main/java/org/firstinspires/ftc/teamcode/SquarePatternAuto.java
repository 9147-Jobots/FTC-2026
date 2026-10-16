package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Autonomous OpMode that resets pose and drives in a 1-meter square pattern.
 */
@Autonomous(name="Square Pattern Auto", group="Linear OpMode")
@SuppressWarnings("unused")
public class SquarePatternAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Follower follower = new Follower(localizer, drivetrain, null);

        // Reset localizer pose to origin (0, 0, 0 rad)
        localizer.setPose(new Pose(0.0, 0.0, 0.0));

        // Leg 1: Forwards 1.0m to (1.0, 0.0)
        Path forwardPath = Paths.line(
            new Pose(0.0, 0.0, 0.0),
            new Pose(1.0, 0.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Leg 2: Right 1.0m to (1.0, -1.0)
        Path rightPath = Paths.line(
            new Pose(1.0, 0.0, 0.0),
            new Pose(1.0, -1.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Leg 3: Backwards 1.0m to (0.0, -1.0)
        Path backPath = Paths.line(
            new Pose(1.0, -1.0, 0.0),
            new Pose(0.0, -1.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Leg 4: Left 1.0m back to origin (0.0, 0.0)
        Path leftPath = Paths.line(
            new Pose(0.0, -1.0, 0.0),
            new Pose(0.0, 0.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Chain the 4 legs sequentially
        Command squareRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, forwardPath),
            new FollowPathCommand(drive, follower, rightPath),
            new FollowPathCommand(drive, follower, backPath),
            new FollowPathCommand(drive, follower, leftPath)
        );

        telemetry.addData("Status", "Initialized Square Pattern Auto");
        telemetry.update();

        waitForStart();

        CommandScheduler.getInstance().schedule(squareRoutine);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetry.update();
        }
    }
}
