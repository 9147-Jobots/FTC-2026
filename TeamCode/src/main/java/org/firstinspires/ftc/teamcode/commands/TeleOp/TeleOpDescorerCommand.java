package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DescorerSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for controlling descorer synchronized servos in TeleOp.
 * - D-Pad UP: Increase descorer angle by +5 degrees on rising edge.
 * - D-Pad DOWN: Decrease descorer angle by -5 degrees on rising edge.
 * - D-Pad RIGHT: Set descorer directly to Maximum position (1.0 / 300°).
 * - D-Pad LEFT: Set descorer directly to Minimum position (0.0 / 0°).
 */
@SuppressWarnings("unused")
public class TeleOpDescorerCommand implements Command {
    private final DescorerSubsystem descorerSubsystem;
    private final Gamepad gamepad;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private static final double FIVE_DEGREES = 5.0;

    public TeleOpDescorerCommand(DescorerSubsystem descorerSubsystem, Gamepad gamepad) {
        this.descorerSubsystem = descorerSubsystem;
        this.gamepad = gamepad;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(descorerSubsystem);
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        // D-pad UP increases descorer angle by 5 degrees on rising edge
        if (gamepad.dpad_up && !lastDpadUp) {
            descorerSubsystem.adjustDescorerAngleDegrees(FIVE_DEGREES);
        }
        lastDpadUp = gamepad.dpad_up;

        // D-pad DOWN decreases descorer angle by 5 degrees on rising edge
        if (gamepad.dpad_down && !lastDpadDown) {
            descorerSubsystem.adjustDescorerAngleDegrees(-FIVE_DEGREES);
        }
        lastDpadDown = gamepad.dpad_down;

        // D-pad RIGHT sets descorer directly to Max Position (1.0 / 300°)
        if (gamepad.dpad_right && !lastDpadRight) {
            descorerSubsystem.setDescorerMax();
        }
        lastDpadRight = gamepad.dpad_right;

        // D-pad LEFT sets descorer directly to Min Position (0.0 / 0°)
        if (gamepad.dpad_left && !lastDpadLeft) {
            descorerSubsystem.setDescorerMin();
        }
        lastDpadLeft = gamepad.dpad_left;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Servos hold position when command ends
    }
}
