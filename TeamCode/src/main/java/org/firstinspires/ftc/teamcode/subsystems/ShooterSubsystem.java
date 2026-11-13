package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;

/**
 * Subsystem encapsulating high-speed shooter flywheel mechanism and 300-degree hood angle servo.
 */
@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
    public static final double HOOD_SERVO_RANGE_DEGREES = 300.0;
    public static final double HOOD_MIN_ANGLE_DEGREES = 150.0;
    public static final double HOOD_MAX_ANGLE_DEGREES = 300.0;

    public static final double HOOD_MIN_POSITION = HOOD_MIN_ANGLE_DEGREES / HOOD_SERVO_RANGE_DEGREES; // 0.5
    public static final double HOOD_MAX_POSITION = HOOD_MAX_ANGLE_DEGREES / HOOD_SERVO_RANGE_DEGREES; // 1.0

    private final Telemetry telemetry;
    private final DcMotorEx shooter;

    private final DcMotorEx kicker;

    private final Servo hoodServo;

    private double targetVelocityTicksPerSec = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Automatically invokes SubsystemBase constructor to handle scheduler registration
        super();

        this.telemetry = telemetry;

        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        this.hoodServo = hardwareMap.get(Servo.class, "hood_servo");
        this.kicker = hardwareMap.get(DcMotorEx.class, "kicker_motor");


        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Default hood position to 300 degrees (max position) on initialization
        setHoodMax();
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter Power", "%4.2f", shooter.getPower());
        telemetry.addData("Shooter Vel (Actual/Target)", "%4.0f / %4.0f TPS", shooter.getVelocity(), targetVelocityTicksPerSec);
        telemetry.addData("Hood Position", "%4.2f", hoodServo.getPosition());
        telemetry.addData("Hood Angle (Deg)", "%4.1f°", getHoodAngleDegrees());
    }

    public void runShooterPower(double power) {
        shooter.setPower(power);
    }

    public void runKickerPower(double power) {
        kicker.setPower(power);
    }

    public void stopKicker() {
        runKickerPower(0.0);
    }

    /** Runs the shooter flywheel at constant PID velocity (encoder ticks/sec). */
    public void runFlywheelVelocity(double ticksPerSec) {
        this.targetVelocityTicksPerSec = ticksPerSec;
        shooter.setVelocity(ticksPerSec);
    }

    /** Returns current measured flywheel velocity in encoder ticks per second. */
    public double getFlywheelVelocity() {
        return shooter.getVelocity();
    }

    /** Sets custom PIDF velocity control coefficients for the shooter motor. */
    public void setFlywheelPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /** Returns current PIDF velocity control coefficients of the shooter motor. */
    public PIDFCoefficients getFlywheelPIDFCoefficients() {
        return shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Sets target flywheel velocity and hood angle from an interpolated state. */
    public void setTargetState(ShooterLookupTable.ShooterTargetState state) {
        runFlywheelVelocity(state.getFlywheelVelocity());
        setHoodAngleDegrees(state.getHoodAngleDegrees());
    }

    public void setHoodPosition(double position) {
        double clampedPosition = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
        hoodServo.setPosition(clampedPosition);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /** Returns current hood angle in physical degrees (150 to 300 degrees). */
    public double getHoodAngleDegrees() {
        return getHoodPosition() * HOOD_SERVO_RANGE_DEGREES;
    }

    /** Sets the hood angle directly in physical degrees (clamped between 150° and 300°). */
    public void setHoodAngleDegrees(double degrees) {
        double clampedDegrees = Math.max(HOOD_MIN_ANGLE_DEGREES, Math.min(HOOD_MAX_ANGLE_DEGREES, degrees));
        setHoodPosition(clampedDegrees / HOOD_SERVO_RANGE_DEGREES);
    }

    /** Adjusts the hood angle by a delta amount in degrees (e.g. +5.0° or -5.0°). */
    public void adjustHoodAngleDegrees(double deltaDegrees) {
        setHoodAngleDegrees(getHoodAngleDegrees() + deltaDegrees);
    }

    /** Adjusts the hood servo position by a delta position step (0.0 to 1.0). */
    public void adjustHoodPosition(double deltaPosition) {
        setHoodPosition(getHoodPosition() + deltaPosition);
    }

    /** Moves the hood servo to its maximum position (1.0 / 300°). */
    public void setHoodMax() {
        setHoodAngleDegrees(HOOD_MAX_ANGLE_DEGREES);
    }

    /** Moves the hood servo to its minimum position (0.5 / 150°). */
    public void setHoodMin() {
        setHoodAngleDegrees(HOOD_MIN_ANGLE_DEGREES);
    }

    public void stopShooter() {
        shooter.setPower(0);
    }

    public void stopFlywheel() {
        stopShooter();
    }
}
