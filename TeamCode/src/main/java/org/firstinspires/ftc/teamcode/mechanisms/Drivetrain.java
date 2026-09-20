package org.firstinspires.ftc.teamcode.mechanisms;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.hardware.sensors.NextIMU;

public class Drivetrain implements Mechanism {

    private final NextMotor frontLeft = new NextMotor("frontLeft");
    private final NextMotor frontRight = new NextMotor("frontRight");
    private final NextMotor backLeft = new NextMotor("backLeft");
    private final NextMotor backRight = new NextMotor("backRight");
    private final NextIMU imu = new NextIMU();

    private double squareInput(double input) {
        return input * input * Math.signum(input);
    }

    public Command resetHeading() { return instant(() -> imu.resetYaw());}

    public Drivetrain(){
        frontLeft.setDirection(NextMotor.Direction.REVERSE);
        backLeft.setDirection(NextMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);

        // assuming rev logo facing up and usb ports facing forwards btw
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
    }
    public void drive(double y, double x, double rx) {

        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setThrottle(frontLeftPower);
        frontRight.setThrottle(frontRightPower);
        backLeft.setThrottle(backLeftPower);
        backRight.setThrottle(backRightPower);


    }

    public double getHeading() {
        return imu.getYaw();
    }
    public void driveFieldCentric(double y, double x, double rx, double heading, double scalar)
    {
        y = squareInput(y);
        x = squareInput(x);
        rx = squareInput(rx);

        double rotX =
                x * Math.cos(-heading)
                        - y * Math.sin(-heading);

        double rotY =
                x * Math.sin(-heading)
                        + y * Math.cos(-heading);

        drive(
                rotY * scalar,
                rotX * scalar,
                rx * scalar
        );
    }



}
