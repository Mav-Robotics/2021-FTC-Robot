package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.subsystems.Capping;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class CappingDriverForward extends CommandBase {
    private Capping m_capping;
    private Telemetry m_telemetry;

    public CappingDriverForward(Capping capping, Telemetry telemetry) {
        m_capping = capping;
        m_telemetry = telemetry;
        addRequirements(m_capping);
    }

    @Override
    public void initialize() {
        m_telemetry.addData("Subsystem CappingDriverForward Command", "Initialized");

    }

    @Override
    public void execute() {
        m_capping.runCapping(RobotMap.CAPPING_SPEED_FORWARD);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
