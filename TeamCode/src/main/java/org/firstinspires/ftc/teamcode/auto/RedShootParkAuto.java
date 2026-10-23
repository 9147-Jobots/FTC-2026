package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.shootPark.RedShootParkPath;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Autonomous OpMode for Red Alliance Shoot & Park (Mirrored).
 */
@Autonomous(name="Red Shoot Park Auto", group="Red Alliance")
@SuppressWarnings("unused")
public class RedShootParkAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        Follower follower = Constants.create(hardwareMap);

        RedShootParkPath autoPaths = new RedShootParkPath();

        // Set starting pose from Red Shoot Park paths
        follower.localizer.setPose(autoPaths.start);

        // Chain the 3 path segments
        Command autoRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, autoPaths.path1()),
            new FollowPathCommand(drive, follower, autoPaths.path2()),
            new FollowPathCommand(drive, follower, autoPaths.path3())
        );

        // Continuously update localizer while waiting for start button
        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addData("Status", "Initialized Red Shoot Park Auto");
            telemetry.addData("X", "%4.2f", follower.pose().x());
            telemetry.addData("Y", "%4.2f", follower.pose().y());
            telemetry.addData("Heading Deg", "%4.2f", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(autoRoutine);

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
