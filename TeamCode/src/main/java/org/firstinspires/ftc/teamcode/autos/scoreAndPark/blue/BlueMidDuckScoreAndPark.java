package org.firstinspires.ftc.teamcode.autos.scoreAndPark.blue;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveBackward;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.commands.StrafeDistance;
import org.firstinspires.ftc.teamcode.commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class BlueMidDuckScoreAndPark extends SequentialCommandGroup {

    public BlueMidDuckScoreAndPark(DrivetrainMecanum drivetrain, Arm arm, Intake intake, Telemetry telemetry, Carousel carousel) {
        addCommands(new ArmToPosition(arm, RobotMap.MID_TARGET, telemetry),
                new DriveDistance(drivetrain, 0.6, 11.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new StrafeDistance(drivetrain, 0.6, 20.5, "RIGHT", telemetry).whenFinished(() -> drivetrain.stopAll()),
                new DriveDistance(drivetrain, 0.4, -2.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new CarouselDriveBackward(carousel, telemetry).withTimeout(3000).whenFinished(() -> carousel.stopAll()),
                new StrafeDistance(drivetrain, 0.6, 21.5, "LEFT", telemetry).whenFinished(() -> drivetrain.stopAll()),
                new ArmToPosition(arm, RobotMap.PICKUP, telemetry).whenFinished(() -> arm.stopAll()),
                new IntakeOut(intake, telemetry).withTimeout(2500).whenFinished(() -> intake.stopIntake()),
                new TurnToAngle(drivetrain, 45.0, 0.5),
                new DriveDistance(drivetrain, 0.6, -21.5, telemetry).whenFinished(() -> drivetrain.stopAll()),
                new StrafeDistance(drivetrain, 0.6, 5.0, "RIGHT", telemetry).whenFinished(() -> drivetrain.stopAll()));

        addRequirements(arm, drivetrain, intake, carousel);
    }
}
