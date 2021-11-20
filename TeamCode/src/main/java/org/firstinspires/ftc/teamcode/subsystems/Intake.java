package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    Telemetry m_telemetry;
    CRServo m_intake;



    public Intake(CRServo intake, Telemetry telemetry) {

        m_telemetry = telemetry;
        m_intake = intake;

        m_telemetry.addLine("Intake Initialized");

    }


    @Override
    public void periodic() {

    }

    public void runIntake(Double speed) {

        m_intake.set(speed);
        m_telemetry.addData("Intake speeds", "Intake: %.2f",
                m_intake.get());
    }

    public void stopIntake() {
        m_intake.set(0.0);
    }
}
