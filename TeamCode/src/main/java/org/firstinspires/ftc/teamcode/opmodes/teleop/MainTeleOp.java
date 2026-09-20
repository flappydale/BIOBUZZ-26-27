package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "BIOBUZZ Teleop")
public class MainTeleOp extends NextOpMode {

    private final Robot robot;

    public MainTeleOp(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start(){
        CommandGamepad driver = new CommandGamepad(gamepad1);

        driver.rightTrigger().isOver(0.1).whileTrue(robot.intake.run());
        driver.rightTrigger().isOver(0.1).onFalse(robot.intake.stop());

        driver.cross().whileTrue(robot.intake.outtake());
        driver.cross().onFalse(robot.intake.stop());

        driver.rightBumper().onTrue(
                sequential(
                        robot.intake.openGate(),
                        robot.intake.run()
                )
        );

        driver.rightBumper().onFalse(
                parallel(
                        robot.intake.stop(),
                        robot.intake.closeGate()
                )
        );

        // reset heading for feild centric view
        driver.start().onTrue(robot.drivetrain.resetHeading());


    }

    @Override
    public void periodic() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double scalar = 1.0;

        if (gamepad1.left_trigger > 0.05) {
            scalar = 0.3;
        }

        robot.drivetrain.driveFieldCentric(y, x , rx, robot.drivetrain.getHeading(), scalar);
    }
}
