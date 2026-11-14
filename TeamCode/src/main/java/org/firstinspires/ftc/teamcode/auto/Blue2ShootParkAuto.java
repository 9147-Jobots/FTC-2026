package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.AutoAimAndShootCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.ParallelDeadlineGroup;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.framework.WaitCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.shootPark.Blue2ShootParkPath;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
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
@Autonomous(name="Blue 2 Shoot Park Auto", group="Blue Alliance")
@SuppressWarnings("unused")
public class Blue2ShootParkAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry);
        AprilTagVisionSubsystem vision = new AprilTagVisionSubsystem(hardwareMap, telemetry);

        Follower follower = Constants.create(hardwareMap);
        Blue2ShootParkPath autoPaths = new Blue2ShootParkPath();

        // Configure Blue Alliance vision target group
        vision.setActiveAlliance(AprilTagTargetGroup.Alliance.BLUE);

        ShooterLookupTable lookupTable = new ShooterLookupTable();

        // Auto Aim Command for Autonomous hardcoded to 0.80 meters distance fallback
        AutoAimAndShootCommand autoAimCommand = new AutoAimAndShootCommand(
                shooter,
                drive,
                indexer,
                gamepad1,
                () -> 0.80, // Hardcoded 0.80m target distance fallback
                () -> vision.getProjectedTargetResult().isTargetDetected() ? vision.getProjectedTargetResult().getYawDegrees() : 0.0,
                () -> true, // Target ready fallback for autonomous shooting
                lookupTable
        );

        // Set starting pose from Blue 2 Shoot Park paths
        follower.localizer.setPose(autoPaths.start);

        // Chain Path 1 -> Path 2 -> 15s Wait -> 5s Auto-Shoot -> Path 3 (Park)
        Command autoRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, autoPaths.path1()),
            new FollowPathCommand(drive, follower, autoPaths.path2()),
            new WaitCommand(15.0),
            new ParallelDeadlineGroup(new WaitCommand(5.0), autoAimCommand),
            new FollowPathCommand(drive, follower, autoPaths.path3())
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
