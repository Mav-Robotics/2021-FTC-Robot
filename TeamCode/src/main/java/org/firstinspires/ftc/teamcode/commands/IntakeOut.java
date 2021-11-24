package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeOut extends CommandBase {
    private Intake m_intake;
    private Telemetry m_telemetry;


    public IntakeOut(Intake intake, Telemetry telemetry) {
        m_intake = intake;
        m_telemetry = telemetry;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.runIntake(RobotMap.INTAKE_SPEED_OUT);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}
