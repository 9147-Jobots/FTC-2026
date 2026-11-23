package org.firstinspires.ftc.teamcode.commands.Auto;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DescorerSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command responsible for controlling descorer synchronized servos in TeleOp.
 * - D-Pad UP: Increase descorer angle by +5 degrees on rising edge.
 * - D-Pad DOWN: Decrease descorer angle by -5 degrees on rising edge.
 * - D-Pad RIGHT: Set descorer directly to Maximum position (0.60 / 180°).
 * - D-Pad LEFT: Set descorer directly to Minimum position (0.25 / 75°).
 */
@SuppressWarnings("unused")
public class AutoDescorerMinCommand implements Command {
    private final DescorerSubsystem descorerSubsystem;

    public AutoDescorerMinCommand(DescorerSubsystem descorerSubsystem) {
        this.descorerSubsystem = descorerSubsystem;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(descorerSubsystem);
    }

    @Override
    public void init() {descorerSubsystem.setDescorerMin();}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Servos hold position when command ends
    }
}
