package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating high-speed shooter flywheel mechanism and hood angle servo.
 */
@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {
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
        telemetry.addData("Hood Position", "%4.2f", hoodServo.getPosition());
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

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public void stopShooter() {
        currentPower = 0.0;
        motor.setPower(0);
    }

    public void stopFlywheel() {
        stopShooter();
    }
}
