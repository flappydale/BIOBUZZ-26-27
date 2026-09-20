package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    public final Intake intake = new Intake();
    public final Drivetrain drivetrain = new Drivetrain();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, drivetrain);
    }


}
