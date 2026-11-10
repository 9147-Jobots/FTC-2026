package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public static final double HOOD_MIN_POSITION = 0.0;
    public static final double HOOD_MAX_POSITION = 1.0;

    private final Telemetry telemetry;
    private final DcMotorEx motor;
    private final Servo hoodServo;

    private double currentPower = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Automatically invokes SubsystemBase constructor to handle scheduler registration
        super();

        this.telemetry = telemetry;

        this.motor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        this.hoodServo = hardwareMap.get(Servo.class, "hood_servo");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter Power", "%4.2f", motor.getPower());
        telemetry.addData("Shooter Velocity (TPS)", "%4.2f", motor.getVelocity());
        telemetry.addData("Hood Position (0-1)", "%4.2f", hoodServo.getPosition());
        telemetry.addData("Hood Angle (Deg)", "%4.1f°", getHoodAngleDegrees());
    }

    public void runShooterPower(double power) {
        currentPower = power;
        motor.setPower(power);
    }

    /** Runs the shooter flywheel at 100% max power. */
    public void runFlywheelMax() {
        runShooterPower(1.0);
    }

    /** Runs the shooter flywheel at constant PID velocity (encoder ticks/sec). */
    public void runFlywheelVelocity(double ticksPerSec) {
        motor.setVelocity(ticksPerSec);
    }

    /** Returns current measured flywheel velocity in encoder ticks per second. */
    public double getFlywheelVelocity() {
        return motor.getVelocity();
    }

    /** Sets target flywheel velocity and hood angle from an interpolated state. */
    public void setTargetState(ShooterLookupTable.ShooterTargetState state) {
        runFlywheelVelocity(state.getFlywheelVelocity());
        setHoodAngleDegrees(state.getHoodAngleDegrees());
    }

    public void setHoodPosition(double position) {
        double clampedPosition = Math.max(0.0, Math.min(1.0, position));
        hoodServo.setPosition(clampedPosition);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /** Returns current hood angle in physical degrees (0 to 300 degrees). */
    public double getHoodAngleDegrees() {
        return getHoodPosition() * HOOD_SERVO_RANGE_DEGREES;
    }

    /** Sets the hood angle directly in physical degrees (0 to 300 degrees). */
    public void setHoodAngleDegrees(double degrees) {
        setHoodPosition(degrees / HOOD_SERVO_RANGE_DEGREES);
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
        setHoodPosition(HOOD_MAX_POSITION);
    }

    /** Moves the hood servo to its minimum position (0.0 / 0°). */
    public void setHoodMin() {
        setHoodPosition(HOOD_MIN_POSITION);
    }

    public void stopShooter() {
        currentPower = 0.0;
        motor.setPower(0);
    }

    public void stopFlywheel() {
        stopShooter();
    }
}
