package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.AutoDescorerMinCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoRunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.framework.ParallelDeadlineGroup;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.framework.WaitCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.flower.BlueFlowerPath;
import org.firstinspires.ftc.teamcode.subsystems.DescorerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Autonomous OpMode for Blue Alliance Park.
 */
@Autonomous(name="Blue Flower Auto", group="Blue Alliance")
@SuppressWarnings("unused")
public class BlueFlowerAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        DescorerSubsystem descorer = new DescorerSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        Follower follower = Constants.create(hardwareMap);

        BlueFlowerPath autoPaths = new BlueFlowerPath();

        // Set starting pose from Blue Park paths
        follower.localizer.setPose(autoPaths.start);

        // Chain the path segments
        Command autoRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, autoPaths.blueToFlower()),
            new AutoDescorerMinCommand(descorer),
            new AutoRunIntakeCommand(intake),
            new WaitCommand(0.5),
            new FollowPathCommand(drive, follower, autoPaths.blueFlowerExtract()),
            new WaitCommand(2),
            new FollowPathCommand(drive, follower, autoPaths.blueFlowerBack())
        );

        // Continuously update localizer while waiting for start button
        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addData("Status", "Initialized Blue Park Auto");
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
