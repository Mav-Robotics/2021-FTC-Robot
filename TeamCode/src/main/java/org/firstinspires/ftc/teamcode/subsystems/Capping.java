package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.ConstantProvider;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Capping extends SubsystemBase {
    Telemetry m_telemetry;
    CRServo m_capping;


    public Capping(CRServo capping, Telemetry telemetry) {

        m_telemetry = telemetry;
        m_capping = capping;

        m_telemetry.addLine("Capping Initialized");

    }


    @Override
    public void periodic() {

    }

    public void runCapping(Double speed) {

        m_capping.set(speed);
        m_telemetry.addData("Intake speeds", "Intake: %.2f",
                m_capping.get());
    }

    public void stopCapping() {
        m_capping.set(0.0);
    }
}
