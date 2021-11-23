package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

public class TurnToAngle extends CommandBase {
    private DrivetrainMecanum m_drivetrain;
    private Double m_angle;
    private Double m_speed;
    private Double m_startAngle;

    public TurnToAngle(DrivetrainMecanum drivetrain, Double angle, Double speed) {
        m_drivetrain = drivetrain;
        m_angle = angle;
        m_speed = speed;

        addRequirements(m_drivetrain);

    }

    @Override
    public void initialize() {
        m_startAngle = m_drivetrain.getCurrentHeading();
    }

    @Override
    public void execute() {
        Double error = Utils.getModulusError(m_angle, m_drivetrain.getCurrentHeading(), -180, 180);

        if (Math.abs(error) > m_speed) {
            error = Math.signum(error) * m_speed;
        }
        m_drivetrain.drive(0, 0, error);
    }

    @Override
    public boolean isFinished() {
        if (m_angle >= 0.0) {
            return (Math.abs(m_angle - m_drivetrain.getCurrentHeading()) <= 1.5);
        } else {
            return (Math.abs(m_angle - m_drivetrain.getCurrentHeading()) >= 1.5);
        }
    }
}
