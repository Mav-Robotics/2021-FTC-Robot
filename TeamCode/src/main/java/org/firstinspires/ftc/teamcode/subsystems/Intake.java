package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    Telemetry m_telemetry;
    MotorEx m_intake;
    Double m_multiplier;



    public Intake(MotorEx intake, Telemetry telemetry) {

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
