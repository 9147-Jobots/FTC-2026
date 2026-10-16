package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Auto.FollowPathCommand;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.framework.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Autonomous OpMode executing exported visualizer paths.
 */
@Autonomous(name="Pedro Generated Auto", group="Linear OpMode")
@SuppressWarnings("unused")
public class PedroGeneratedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Follower follower = new Follower(localizer, drivetrain, null);

        AutoPaths autoPaths = new AutoPaths();

        // Set starting pose from exported paths
        localizer.setPose(autoPaths.start);

        // Chain the exported path segments
        Command autoRoutine = new SequentialCommandGroup(
            new FollowPathCommand(drive, follower, autoPaths.path1()),
            new FollowPathCommand(drive, follower, autoPaths.path2())
        );

        telemetry.addData("Status", "Initialized Pedro Generated Auto");
        telemetry.update();

        waitForStart();

        CommandScheduler.getInstance().schedule(autoRoutine);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            telemetry.update();
        }
    }
}
