package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
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
        Follower follower = Constants.create(hardwareMap);

        // Reset localizer pose to origin (0, 0, 0 rad)
        follower.localizer.setPose(new Pose(0.0, 0.0, 0.0));

        // Leg 1: Forwards 24 inches to (24.0, 0.0)
        Path forwardPath = Paths.line(
            new Pose(0.0, 0.0, 0.0),
            new Pose(24.0, 0.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Leg 2: Right 24 inches to (24.0, -24.0)
        Path rightPath = Paths.line(
            new Pose(24.0, 0.0, 0.0),
            new Pose(24.0, -24.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Leg 3: Backwards 24 inches to (0.0, -24.0)
        Path backPath = Paths.line(
            new Pose(24.0, -24.0, 0.0),
            new Pose(0.0, -24.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Leg 4: Left 24 inches back to origin (0.0, 0.0)
        Path leftPath = Paths.line(
            new Pose(0.0, -24.0, 0.0),
            new Pose(0.0, 0.0, 0.0)
        ).constant(Math.toRadians(0.0));

        // Chain the 4 legs sequentially
        Command squareRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, forwardPath),
            new FollowPathCommand(drive, follower, rightPath),
            new FollowPathCommand(drive, follower, backPath),
            new FollowPathCommand(drive, follower, leftPath)
        );

        // Continuously update localizer while waiting for start button
        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addData("Status", "Initialized Square Pattern Auto");
            telemetry.addData("X", "%4.2f", follower.pose().x());
            telemetry.addData("Y", "%4.2f", follower.pose().y());
            telemetry.addData("Heading Deg", "%4.2f", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(squareRoutine);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("Is Following", follower.following());
            telemetry.addData("X", "%4.2f", follower.pose().x());
            telemetry.addData("Y", "%4.2f", follower.pose().y());
            telemetry.addData("Heading Deg", "%4.2f", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }
}
