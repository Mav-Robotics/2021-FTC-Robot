package org.firstinspires.ftc.teamcode.autos.scoreFromStart.blue;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class BlueLowScore extends SequentialCommandGroup {

    public BlueLowScore(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry) {
        addCommands(
                new ArmToPosition(arm, RobotMap.LOW_TARGET, telemetry),
                new TurnToAngle(drivetrain, -9.0, 0.5),
                new DriveDistance(drivetrain, 0.6, 18.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new IntakeOut(intake, telemetry).withTimeout(2000).whenFinished(() -> intake.stopIntake()),
                new DriveDistance(drivetrain, 0.6, -18.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new ArmToPosition(arm, RobotMap.PICKUP, telemetry).whenFinished(() -> arm.stopAll()),
                new TurnToAngle(drivetrain, 0.0, 0.5)
                );

        addRequirements(arm, drivetrain, intake);
    }
}
