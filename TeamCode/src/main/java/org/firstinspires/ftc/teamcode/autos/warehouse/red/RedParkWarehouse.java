package org.firstinspires.ftc.teamcode.autos.warehouse.red;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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

import java.util.Locale;

public class RedParkWarehouse extends SequentialCommandGroup {

    public RedParkWarehouse(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry) {

        addCommands(
                new DriveDistance(drivetrain, 0.6, 1.0, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new TurnToAngle(drivetrain, -90.0, 0.5),
                new StrafeDistance(drivetrain, 0.6, 3.0, "RIGHT", telemetry).whenFinished(() -> drivetrain.stopAll()),
                new DriveDistance(drivetrain, 0.6, 38.0, telemetry).whenFinished(() -> drivetrain.stopAll())
                );

        addRequirements(arm, drivetrain, intake);
    }
}
