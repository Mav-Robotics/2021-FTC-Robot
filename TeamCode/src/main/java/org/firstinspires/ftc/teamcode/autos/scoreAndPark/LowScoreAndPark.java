package org.firstinspires.ftc.teamcode.autos.scoreAndPark;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class LowScoreAndPark extends SequentialCommandGroup {
    static final Integer PICKUP = 0;
    static final Integer LOW_TARGET = 585;
    static final Integer MID_TARGET = 1220;
    static final Integer HI_TARGET = 1800;

    public LowScoreAndPark(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry) {
        addCommands(
                new ArmToPosition(arm, LOW_TARGET, telemetry),
                new TurnToAngle(drivetrain, 27.0, 0.35),
                new DriveDistance(drivetrain, 0.5, 28.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new IntakeOut(intake, telemetry).withTimeout(2000).whenFinished(() -> intake.stopIntake()),
                new DriveDistance(drivetrain, 0.5, -10.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new ArmToPosition(arm, PICKUP, telemetry).whenFinished(() -> arm.stopAll()),
                new TurnToAngle(drivetrain, 0.0, 0.35),
                new TurnToAngle(drivetrain, -90.0, 0.35)
        );

        addRequirements(arm, drivetrain, intake);
    }
}
