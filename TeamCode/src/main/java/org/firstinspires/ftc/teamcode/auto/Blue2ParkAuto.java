package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.park.Blue2ParkPath;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Autonomous OpMode for Blue 2 Alliance Park.
 */
@Autonomous(name="Blue 2 Park Auto", group="Blue Alliance")
@SuppressWarnings("unused")
public class Blue2ParkAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        Follower follower = Constants.create(hardwareMap);

        Blue2ParkPath autoPaths = new Blue2ParkPath();

        // Set starting pose from Blue 2 Park paths
        follower.localizer.setPose(autoPaths.start);

        // Chain the path segment
        Command autoRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, autoPaths.path1())
        );

        // Continuously update localizer while waiting for start button
        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addData("Status", "Initialized Blue 2 Park Auto");
            telemetry.addData("X", "%4.2f", follower.pose().x());
            telemetry.addData("Y", "%4.2f", follower.pose().y());
            telemetry.addData("Heading Deg", "%4.2f", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(autoRoutine);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            org.firstinspires.ftc.teamcode.util.PoseStorage.currentPose = follower.pose();
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("Is Following", follower.following());
            telemetry.addData("X", "%4.2f", follower.pose().x());
            telemetry.addData("Y", "%4.2f", follower.pose().y());
            telemetry.addData("Heading Deg", "%4.2f", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }
}
