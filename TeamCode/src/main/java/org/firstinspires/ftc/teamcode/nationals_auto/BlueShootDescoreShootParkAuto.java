package org.firstinspires.ftc.teamcode.nationals_auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.AutoDescorerMaxCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoDescorerMinCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoRunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoStopIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.AutoAimAndShootCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.ParallelDeadlineGroup;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.framework.WaitCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.nationals_paths.BlueFlowerParkPath;
import org.firstinspires.ftc.teamcode.pedroPathing.nationals_paths.BlueFlowerShootParkPath;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DescorerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.vision.AprilTagTargetGroup;

/**
 * Autonomous OpMode for Blue 2 Alliance Shoot & Park.
 * Sequence:
 * 1. Drives Path 1 and Path 2 to end of 2nd path.
 * 2. Pauses and waits 15 seconds.
 * 3. Runs AutoAimAndShootCommand for 5 seconds.
 * 4. Drives Path 3 to park.
 */
@Autonomous(name="Blue Shoot Descore Auto", group="Blue Alliance")
@SuppressWarnings("unused")
public class BlueShootDescoreShootParkAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        AprilTagVisionSubsystem vision = new AprilTagVisionSubsystem(hardwareMap, telemetry);
        DescorerSubsystem descorer = new DescorerSubsystem(hardwareMap, telemetry);

        Follower follower = Constants.create(hardwareMap);
        BlueFlowerShootParkPath autoPaths = new BlueFlowerShootParkPath();

        // Configure Blue Alliance vision target group
        vision.setActiveAlliance(AprilTagTargetGroup.Alliance.BLUE);

        ShooterLookupTable lookupTable = new ShooterLookupTable();

        // Auto Aim Command for Autonomous (uses vision when target detected, defaults to 1.1m fallback)
        AutoAimAndShootCommand autoAimCommand = new AutoAimAndShootCommand(
                shooter, drive, indexer, intake, vision, null, gamepad1, lookupTable
        );

        // Set starting pose from Blue 2 Shoot Park paths
        follower.localizer.setPose(autoPaths.start);

        // Chain Path 1 -> Path 2 -> 15s Wait -> 5s Auto-Shoot -> Path 3 (Park)
        Command autoRoutine = new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(5.0), autoAimCommand),
                new FollowPathCommand(drive, follower, autoPaths.descore1()),
                new FollowPathCommand(drive, follower, autoPaths.descore2()),
                new FollowPathCommand(drive, follower, autoPaths.descore3()),
                new AutoDescorerMinCommand(descorer),
                new WaitCommand(0.5),
                new AutoRunIntakeCommand(intake, indexer),
                new ParallelDeadlineGroup(new WaitCommand(2), new SequentialCommandGroup(new FollowPathCommand(drive, follower, autoPaths.descoreExtract().with(Constants.foresightConfig.maxPathSpeed.at(0.4))), new WaitCommand(2))),
                new FollowPathCommand(drive, follower, autoPaths.descorerBack()),
                new AutoDescorerMaxCommand(descorer),
                new AutoStopIntakeCommand(intake, indexer),
                new FollowPathCommand(drive, follower, autoPaths.shoot2Prep1()),
                new FollowPathCommand(drive, follower, autoPaths.shootPrep2()),
                new FollowPathCommand(drive, follower, autoPaths.shootPrep3()),
                new ParallelDeadlineGroup(new WaitCommand(5.0), autoAimCommand),
                new FollowPathCommand(drive, follower, autoPaths.park())
        );

        // Continuously update localizer while waiting for start button
        while (!isStarted() && !isStopRequested()) {
            follower.localizer.update();
            telemetry.addData("Status", "Initialized Blue 2 Shoot Park Auto");
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

        vision.stopVision();
    }
}
