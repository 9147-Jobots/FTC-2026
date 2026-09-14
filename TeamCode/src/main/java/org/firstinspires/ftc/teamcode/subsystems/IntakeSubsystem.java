package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating a 4-motor Omni-Directional / Mecanum drivetrain.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final double intakePower = 1;
    private final Telemetry telemetry;
    private final DcMotor intakeMotor;


    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Automatically invokes SubsystemBase constructor to handle scheduler registration
        super();

        this.telemetry = telemetry;

        // Perform hardware initializations directly inside the constructor
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        // Subsystem periodic can be used for sensor monitoring or telemetry health updates!
        telemetry.addData("Intake Power", "%4.2f", intakePower);
    }

    /**
     * Controls the wheels given directional power components.
     */
    public void runIntake() {
        intakeMotor.setPower(intakePower);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
