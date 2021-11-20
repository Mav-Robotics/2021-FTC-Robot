package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmDefaultDrive extends CommandBase {
    private Arm m_arm;
    private Telemetry m_telemetry;
    private GamepadEx m_gamepad;

    public ArmDefaultDrive(Arm arm, GamepadEx gamepad, Telemetry telemetry) {
        m_arm = arm;
        m_telemetry = telemetry;
        m_gamepad = gamepad;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double stick_val = m_gamepad.getLeftY();
        if (stick_val >= 0.25) {
            m_arm.drive(1.0);
        } else if (stick_val <= -0.25) {
            m_arm.drive(-1.0);
        } else {
            m_arm.drive(0.0);
        }


    }

    @Override
    public boolean isFinished() { return false; }
}
