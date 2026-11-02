package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    private final DcMotor motor;

    private final Servo hoodServo;


    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Automatically invokes SubsystemBase constructor to handle scheduler registration
        super();

        this.telemetry = telemetry;

        // Perform hardware initializations directly inside the constructor
        this.motor = hardwareMap.get(DcMotor.class, "shooter_motor");

        this.hoodServo = hardwareMap.get(Servo.class, "hood_servo");

        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        // Subsystem periodic can be used for sensor monitoring or telemetry health updates!
        telemetry.addData("Shooter Power", "%4.2f", motor.getPower());
    }

    public void runShooterPower(double power) {
        motor.setPower(power);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public void stopShooter() {
        motor.setPower(0);
    }
}
