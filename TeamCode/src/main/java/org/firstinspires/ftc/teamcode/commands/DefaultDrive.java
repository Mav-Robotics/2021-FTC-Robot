package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

public class DefaultDrive extends CommandBase {
    private DrivetrainMecanum m_drivetrain;
    private GamepadEx m_driver;
    private Telemetry m_telemetry;

    public DefaultDrive(DrivetrainMecanum drivetrain, GamepadEx driver, Telemetry telemetry) {
        m_drivetrain = drivetrain;
        m_driver = driver;
        m_telemetry = telemetry;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drivetrain.drive(m_driver.getRightX(), -m_driver.getRightY(), -m_driver.getLeftX());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
