package org.firstinspires.ftc.teamcode.autos.scoreAndPark.red;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.commands.StrafeDistance;
import org.firstinspires.ftc.teamcode.commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class RedScoreAndParkWarehouse extends SequentialCommandGroup {

    public RedScoreAndParkWarehouse(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry) {

        addCommands(
                new ArmToPosition(arm, RobotMap.MID_TARGET, telemetry),
                new TurnToAngle(drivetrain, 9.0, 0.5),
                new DriveDistance(drivetrain, 0.6, 19.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new IntakeOut(intake, telemetry).withTimeout(2000).whenFinished(() -> intake.stopIntake()),
                new DriveDistance(drivetrain, 0.6, -19.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new DriveDistance(drivetrain, 0.6, 2.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new TurnToAngle(drivetrain, -90.0, 0.5),
                new StrafeDistance(drivetrain, 0.6, 2.0, "RIGHT", telemetry).whenFinished(() -> drivetrain.stopAll()),
                new DriveDistance(drivetrain, 0.6, 38.0, telemetry).whenFinished(() -> drivetrain.stopAll())
                );

        addRequirements(arm, drivetrain, intake);
    }
}
