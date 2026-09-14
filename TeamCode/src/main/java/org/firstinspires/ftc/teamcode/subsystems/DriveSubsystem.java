package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating a 4-motor Omni-Directional / Mecanum drivetrain.
 */
public class DriveSubsystem extends SubsystemBase {
    private final Telemetry telemetry;

    private final IMU imu;

    private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Automatically invokes SubsystemBase constructor to handle scheduler registration
        super();

        this.telemetry = telemetry;

        // Perform hardware initializations directly inside the constructor

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        // Drive
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        // Subsystem periodic can be used for sensor monitoring or telemetry health updates!
    }

    /**
     * Controls the wheels given directional power components.
     */
    public void drive(double axial, double lateral, double yaw) {

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        double frontLeftPower  = rotY + rotX + yaw;
        double frontRightPower = rotY - rotX - yaw;
        double backLeftPower   = rotY - rotX + yaw;
        double backRightPower  = rotY + rotX - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}
