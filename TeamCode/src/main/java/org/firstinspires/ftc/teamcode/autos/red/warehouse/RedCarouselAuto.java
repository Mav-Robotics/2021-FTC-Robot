package org.firstinspires.ftc.teamcode.autos.red.warehouse;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveBackward;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.StrafeDistance;
import org.firstinspires.ftc.teamcode.commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class RedCarouselAuto extends SequentialCommandGroup {

    public RedCarouselAuto(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry) {
        addCommands(
                new DriveDistance(drivetrain, 0.6, 7.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new StrafeDistance(drivetrain, 0.6, 22.5, "RIGHT", telemetry).whenFinished(() -> drivetrain.stopAll()),
                new CarouselDriveForward(drivetrain, )
                new DriveDistance(drivetrain, 0.6, 38.0, telemetry).whenFinished(() -> drivetrain.stopAll())
                );

        addRequirements(arm, drivetrain, intake);
    }
}
