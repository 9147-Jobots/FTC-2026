package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.Battery;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;

/**
 * Subsystem encapsulating high-speed shooter flywheel mechanism with custom voltage-compensated feedforward/PID,
 * kicker motor with non-blocking pulsing, and 300-degree hood angle servo.
 */
@Config
@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
    public static double kS = 0.5;                  // Static friction feedforward offset (volts)
    public static double kV = 0.0035;               // Velocity feedforward gain (volts per TPS)
    public static double kP = 0.0010;               // Proportional feedback gain
    public static double kI = 0.0;                  // Integral feedback gain
    public static double VELOCITY_TOLERANCE_TPS = 100.0;     // Velocity error tolerance threshold for ready check
    public static double READY_TIME_THRESHOLD_MS = 100.0;    // Required duration error must stay within tolerance (ms)
    public static double VELOCITY_EMA_ALPHA = 0.25;          // Low-pass filter smoothing coefficient

    public static final double HOOD_SERVO_RANGE_DEGREES = 300.0;
    public static final double HOOD_MIN_ANGLE_DEGREES = 140.0;
    public static final double HOOD_MAX_ANGLE_DEGREES = 270.0;

    public static final double HOOD_MIN_POSITION = HOOD_MIN_ANGLE_DEGREES / HOOD_SERVO_RANGE_DEGREES; // 0.5
    public static final double HOOD_MAX_POSITION = HOOD_MAX_ANGLE_DEGREES / HOOD_SERVO_RANGE_DEGREES; // 1.0

    private final Telemetry telemetry;
    private final DcMotorEx shooter;
    private final DcMotorEx kicker;
    private final Servo hoodServo;
    private final Battery battery;

    private double targetVelocityTicksPerSec = 0.0;
    private double filteredVelocityTPS = 0.0;
    private double integralError = 0.0;

    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private final ElapsedTime kickerPulseTimer = new ElapsedTime();
    private boolean isKickerPulsing = false;
    private double kickerPulseOnMs = 200.0;
    private double kickerPulseOffMs = 150.0;
    private double kickerPulsePower = 1.0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super();

        this.telemetry = telemetry;
        this.battery = new Battery(hardwareMap);

        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        this.hoodServo = hardwareMap.get(Servo.class, "hood_servo");
        this.kicker = hardwareMap.get(DcMotorEx.class, "kicker_motor");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        loopTimer.reset();
        readyTimer.reset();

        // Default hood position to 300 degrees (max position) on initialization
        setHoodMax();
    }

    /** Starts non-blocking kicker pulsing with custom ON/OFF durations (in ms) and power against continuous subsystem clock. */
    public void startKickerPulsing(double onMs, double offMs, double power) {
        this.kickerPulseOnMs = onMs;
        this.kickerPulseOffMs = offMs;
        this.kickerPulsePower = power;
        this.isKickerPulsing = true;
    }

    /** Starts non-blocking kicker pulsing with defaults (200ms ON / 150ms OFF @ 1.0 power). */
    public void startKickerPulsing() {
        startKickerPulsing(200.0, 150.0, 1.0);
    }

    public boolean isKickerPulsing() {
        return isKickerPulsing;
    }

    @Override
    public void periodic() {
        // 1. Update measured flywheel velocity low-pass filter
        double rawVel = shooter.getVelocity();
        filteredVelocityTPS = (VELOCITY_EMA_ALPHA * rawVel) + ((1.0 - VELOCITY_EMA_ALPHA) * filteredVelocityTPS);

        double dt = loopTimer.seconds();
        loopTimer.reset();

        // 2. Flywheel open-loop feedforward + feedback control loop
        if (targetVelocityTicksPerSec > 0.0) {
            double error = targetVelocityTicksPerSec - filteredVelocityTPS;

            if (dt > 0.0 && dt < 0.2) {
                integralError += error * dt;
            }

            double voltsFF = (kS * Math.signum(targetVelocityTicksPerSec)) + (kV * targetVelocityTicksPerSec);
            double voltsFB = (kP * error) + (kI * integralError);
            double requestedVolts = voltsFF + voltsFB;

            double batteryVolts = battery.volts();
            double power = Math.max(-1.0, Math.min(1.0, requestedVolts / batteryVolts));

            shooter.setPower(power);

            // Track readiness timing
            if (Math.abs(error) > VELOCITY_TOLERANCE_TPS) {
                readyTimer.reset();
            }
        } else {
            shooter.setPower(0.0);
            integralError = 0.0;
            readyTimer.reset();
        }

        // 3. Kicker non-blocking pulsing logic
        if (isKickerPulsing) {
            double totalCycleMs = kickerPulseOnMs + kickerPulseOffMs;
            double elapsedInCycleMs = kickerPulseTimer.milliseconds() % totalCycleMs;

            if (elapsedInCycleMs < kickerPulseOnMs) {
                kicker.setPower(kickerPulsePower);
            } else {
                kicker.setPower(0.0);
            }
        }

        // 4. Telemetry logging
        telemetry.addData("Shooter Power", "%4.2f", shooter.getPower());
        telemetry.addData("Shooter Vel (Filtered/Target)", "%4.0f / %4.0f TPS", filteredVelocityTPS, targetVelocityTicksPerSec);
        telemetry.addData("Shooter Vel Raw", "%4.0f TPS", rawVel);
        telemetry.addData("Battery Volts", "%4.2f V", battery.volts());
        telemetry.addData("Shooter Ready", isReady());
        telemetry.addData("Kicker Power", "%4.2f", kicker.getPower());
        telemetry.addData("Kicker Pulsing", isKickerPulsing);
        telemetry.addData("Hood Position", "%4.2f", hoodServo.getPosition());
        telemetry.addData("Hood Angle (Deg)", "%4.1f°", getHoodAngleDegrees());
    }

    /** Returns true when flywheel velocity error has stayed continuously under tolerance threshold for at least 100 ms. */
    public boolean isReady() {
        if (targetVelocityTicksPerSec <= 0.0) {
            return false;
        }
        double error = Math.abs(targetVelocityTicksPerSec - filteredVelocityTPS);
        return error <= VELOCITY_TOLERANCE_TPS && readyTimer.milliseconds() >= READY_TIME_THRESHOLD_MS;
    }

    public void runShooterPower(double power) {
        shooter.setPower(power);
    }

    public void runKickerPower(double power) {
        this.isKickerPulsing = false;
        kicker.setPower(power);
    }

    public void stopKicker() {
        this.isKickerPulsing = false;
        kicker.setPower(0.0);
    }

    /** Sets target flywheel velocity (encoder ticks/sec). Loop in periodic() will compute voltage feedforward & feedback. */
    public void runFlywheelVelocity(double ticksPerSec) {
        if (Math.abs(ticksPerSec - this.targetVelocityTicksPerSec) > 1.0) {
            integralError = 0.0;
            readyTimer.reset();
        }
        this.targetVelocityTicksPerSec = ticksPerSec;
    }

    /** Sets target flywheel RPM / ticks per sec. */
    public void runShooterRPM(double rpm) {
        runFlywheelVelocity(rpm);
    }

    /** Returns current low-pass filtered flywheel velocity in encoder ticks per second. */
    public double getFlywheelVelocity() {
        return filteredVelocityTPS;
    }

    /** Returns raw unfiltered measured flywheel velocity in encoder ticks per second. */
    public double getRawFlywheelVelocity() {
        return shooter.getVelocity();
    }

    /** Returns target flywheel velocity in encoder ticks per second. */
    public double getTargetVelocityTicksPerSec() {
        return targetVelocityTicksPerSec;
    }

    /** Sets target flywheel velocity and hood angle from an interpolated state. */
    public void setTargetState(ShooterLookupTable.ShooterTargetState state) {
        runFlywheelVelocity(state.getFlywheelVelocity() + 20);
        setHoodAngleDegrees(state.getHoodAngleDegrees());
    }

    public void setHoodPosition(double position) {
        double clampedPosition = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, position));
        hoodServo.setPosition(clampedPosition);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /** Returns current hood angle in physical degrees (140 to 270 degrees). */
    public double getHoodAngleDegrees() {
        return getHoodPosition() * HOOD_SERVO_RANGE_DEGREES;
    }

    /** Sets the hood angle directly in physical degrees (clamped between 140° and 270°). */
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

    /** Moves the hood servo to its maximum operating position (270°). */
    public void setHoodMax() {
        setHoodAngleDegrees(HOOD_MAX_ANGLE_DEGREES);
    }

    /** Moves the hood servo to its minimum position (140°). */
    public void setHoodMin() {
        setHoodAngleDegrees(HOOD_MIN_ANGLE_DEGREES);
    }

    /** Moves the hood servo directly to full 300° resting/hard-stop position to reset dislodged gears or teeth. */
    public void resetHoodTo300Degrees() {
        hoodServo.setPosition(300.0 / HOOD_SERVO_RANGE_DEGREES);
    }

    public void stopShooter() {
        this.targetVelocityTicksPerSec = 0.0;
        this.integralError = 0.0;
        shooter.setPower(0);
    }

    public void stopFlywheel() {
        stopShooter();
    }
}
