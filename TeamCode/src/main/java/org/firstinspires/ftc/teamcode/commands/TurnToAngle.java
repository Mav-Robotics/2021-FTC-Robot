package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

public class TurnToAngle extends CommandBase {
    private DrivetrainMecanum m_drivetrain;
    private Telemetry m_telemetry;
    private Double m_angle;

    public TurnToAngle(DrivetrainMecanum drivetrain, Double angle, Telemetry telemetry) {
        m_drivetrain = drivetrain;
        m_telemetry = telemetry;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() { return false; }
}
