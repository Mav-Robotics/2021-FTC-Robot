package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

public class DriveDistance extends CommandBase {
    private DrivetrainMecanum m_drivetrain;
    private Telemetry m_telemetry;
    private Double m_speed;
    private Double m_distance;

    public DriveDistance(DrivetrainMecanum drivetrain, Double speed, Double distance, Telemetry telemetry) {
        m_drivetrain = drivetrain;
        m_telemetry = telemetry;
        m_speed = speed;
        m_distance = distance;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetHeading();
    }

    @Override
    public void execute() {
        if (m_distance >= 0.0) {
            m_drivetrain.drive(0, -m_speed, 0);
        } else {
            m_drivetrain.drive(0, m_speed, 0);
        }

    }

    @Override
    public boolean isFinished() {
        if (m_distance >= 0.0) {
            return (m_drivetrain.getAverageDistance() >= m_distance);
        } else {
            return (m_drivetrain.getAverageDistance() <= m_distance);
        }
    }
}
