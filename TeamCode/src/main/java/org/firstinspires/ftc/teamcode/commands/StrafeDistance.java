package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

import java.util.Locale;

public class StrafeDistance extends CommandBase {
    private DrivetrainMecanum m_drivetrain;
    private Telemetry m_telemetry;
    private Double m_speed, m_distance;
    private String m_direction;

    public StrafeDistance(DrivetrainMecanum drivetrain, Double speed, Double distance, String direction, Telemetry telemetry) {
        m_drivetrain = drivetrain;
        m_telemetry = telemetry;
        m_speed = speed;
        m_distance = distance;
        m_direction = direction.toLowerCase(Locale.ROOT);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetHeading();
    }

    @Override
    public void execute() {
        if (m_direction.equals("left")) {
            m_drivetrain.drive(m_speed, 0, 0);
        } else if (m_direction.equals("right")){
            m_drivetrain.drive(-m_speed, 0, 0);
        }

    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_drivetrain.getAverageDistance()) > m_distance;
    }
}
