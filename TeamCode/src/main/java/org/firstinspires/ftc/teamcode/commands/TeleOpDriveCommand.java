package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Command responsible for mapping joystick actions dynamically to the robot drivetrain.
 */
public class TeleOpDriveCommand implements Command {
    private final DriveSubsystem drive;
    private final Gamepad gamepad;

    public TeleOpDriveCommand(DriveSubsystem drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        double axial   = -gamepad.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad.left_stick_x;
        double yaw     =  gamepad.right_stick_x;

        if (gamepad.options) {
            drive.resetHeading();
        }
        
        drive.drive(axial, lateral, yaw);
    }

    @Override
    public boolean isFinished() {
        return false; // TeleOp commands run continuously until explicitly stopped by an operator
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}
