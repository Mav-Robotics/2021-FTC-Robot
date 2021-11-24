package org.firstinspires.ftc.teamcode.autos.scoreAndPark;
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

public class LowScoreAndPark extends SequentialCommandGroup {

    public LowScoreAndPark(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry) {
        addCommands(
                new ArmToPosition(arm, RobotMap.LOW_TARGET, telemetry),
                new TurnToAngle(drivetrain, 27.0, 0.5),
                new DriveDistance(drivetrain, 0.6, 28.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new IntakeOut(intake, telemetry).withTimeout(2000).whenFinished(() -> intake.stopIntake()),
                new DriveDistance(drivetrain, 0.6, -10.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new ArmToPosition(arm, RobotMap.PICKUP, telemetry).whenFinished(() -> arm.stopAll()),
                new TurnToAngle(drivetrain, -90.0, 0.5),
                new DriveDistance(drivetrain, 0.6, -31.0, telemetry).whenFinished(() -> drivetrain.stopAll())

                );

        addRequirements(arm, drivetrain, intake);
    }
}
