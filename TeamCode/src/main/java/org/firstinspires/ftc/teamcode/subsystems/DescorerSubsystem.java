package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating dual synchronized 300-degree servos for the descorer mechanism.
 * One servo is set to REVERSE direction so both sides operate synchronously when mounted symmetrically.
 */
@SuppressWarnings("unused")
public class DescorerSubsystem extends SubsystemBase {
    public static final double DESCORER_SERVO_RANGE_DEGREES = 300.0;
    public static final double DESCORER_MIN_ANGLE_DEGREES = 0.0;
    public static final double DESCORER_MAX_ANGLE_DEGREES = 300.0;

    public static final double DESCORER_MIN_POSITION = DESCORER_MIN_ANGLE_DEGREES / DESCORER_SERVO_RANGE_DEGREES; // 0.0
    public static final double DESCORER_MAX_POSITION = DESCORER_MAX_ANGLE_DEGREES / DESCORER_SERVO_RANGE_DEGREES; // 1.0

    private final Telemetry telemetry;
    private final Servo leftServo;
    private final Servo rightServo;

    public DescorerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Automatically registers with CommandScheduler
        super();

        this.telemetry = telemetry;

        this.leftServo = hardwareMap.get(Servo.class, "arm_left_servo");
        this.rightServo = hardwareMap.get(Servo.class, "arm_right_servo");

        // Sync left and right servos by setting opposite directions for mirrored mounting
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        // Default to minimum position on initialization
        setDescorerMin();
    }

    @Override
    public void periodic() {
        telemetry.addData("Descorer Pos (L/R)", "%4.2f / %4.2f", leftServo.getPosition(), rightServo.getPosition());
        telemetry.addData("Descorer Angle (Deg)", "%4.1f°", getDescorerAngleDegrees());
    }

    /** Sets the normalized position (0.0 to 1.0) for both synchronized servos. */
    public void setDescorerPosition(double position) {
        double clampedPosition = Math.max(DESCORER_MIN_POSITION, Math.min(DESCORER_MAX_POSITION, position));
        leftServo.setPosition(clampedPosition);
        rightServo.setPosition(clampedPosition);
    }

    /** Returns current normalized servo position (0.0 to 1.0). */
    public double getDescorerPosition() {
        return leftServo.getPosition();
    }

    /** Returns current descorer angle in physical degrees (0° to 300°). */
    public double getDescorerAngleDegrees() {
        return getDescorerPosition() * DESCORER_SERVO_RANGE_DEGREES;
    }

    /** Sets the descorer angle directly in physical degrees (clamped between 0° and 300°). */
    public void setDescorerAngleDegrees(double degrees) {
        double clampedDegrees = Math.max(DESCORER_MIN_ANGLE_DEGREES, Math.min(DESCORER_MAX_ANGLE_DEGREES, degrees));
        setDescorerPosition(clampedDegrees / DESCORER_SERVO_RANGE_DEGREES);
    }

    /** Adjusts the descorer angle by a delta amount in degrees (e.g. +5.0° or -5.0°). */
    public void adjustDescorerAngleDegrees(double deltaDegrees) {
        setDescorerAngleDegrees(getDescorerAngleDegrees() + deltaDegrees);
    }

    /** Adjusts the descorer servo position by a delta normalized position step. */
    public void adjustDescorerPosition(double deltaPosition) {
        setDescorerPosition(getDescorerPosition() + deltaPosition);
    }

    /** Moves both descorer servos to maximum position (1.0 / 300°). */
    public void setDescorerMax() {
        setDescorerAngleDegrees(DESCORER_MAX_ANGLE_DEGREES);
    }

    /** Moves both descorer servos to minimum position (0.0 / 0°). */
    public void setDescorerMin() {
        setDescorerAngleDegrees(DESCORER_MIN_ANGLE_DEGREES);
    }
}
