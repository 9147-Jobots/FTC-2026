package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    }

    public void runShooterPower(double power) {
        shooter.setPower(power);
    }

    public void runKickerPower(double power) {
        kicker.setPower(power);
    }

    /** Runs the shooter flywheel at constant PID velocity (encoder ticks/sec). */
    public void runFlywheelVelocity(double ticksPerSec) {
        shooter.setVelocity(ticksPerSec);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public void stopShooter() {
        shooter.setPower(0);
    }

    public void stopFlywheel() {
        stopShooter();
    }
}
