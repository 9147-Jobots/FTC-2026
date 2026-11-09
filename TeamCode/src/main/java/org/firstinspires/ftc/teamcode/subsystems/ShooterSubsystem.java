package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating high-speed shooter flywheel mechanism and 300-degree hood angle servo.
 */
@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
    public static final double HOOD_SERVO_RANGE_DEGREES = 300.0;
    public static final double HOOD_MIN_POSITION = 0.0;
    public static final double HOOD_MAX_POSITION = 1.0;

    private final Telemetry telemetry;
    private final DcMotorEx shooter;

    private final DcMotorEx kicker;

    private final Servo hoodServo;

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
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter Power", "%4.2f", shooter.getPower());
        telemetry.addData("Shooter Velocity (TPS)", "%4.2f", shooter.getVelocity());
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
        shooter.setVelocity(ticksPerSec);
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
        shooter.setPower(0);
    }

    public void stopFlywheel() {
        stopShooter();
    }
}
