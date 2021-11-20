package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    Telemetry m_telemetry;
    Double kS, kCos, kV, kA;
    PIDController m_pid;
    MotorEx m_armMotor;


    public Arm(MotorEx armMotor, Telemetry telemetry) {

        m_armMotor = armMotor;
        m_telemetry = telemetry;
        kS = 0.0;
        kCos = 0.0;
        kV = 0.0;
        kA = 0.0;

        ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        telemetry.addData("Arm Values", "kS: %f, kCos: %f, kV: %f, kA: %f", kS, kCos, kV, kA);


        m_telemetry.addLine("Arm Initialized");

    }


    @Override
    public void periodic() {
        m_telemetry.addData("Arm Motor Clicks", m_armMotor.getCurrentPosition());

    }

    public void driveToSetPoint(Double setpoint) {
        m_pid.setSetPoint(setpoint);
        m_telemetry.addData("Arm Setpoint", m_pid.getSetPoint());
        m_telemetry.addData("Arm Position", m_armMotor.getCurrentPosition());
    }

    public void drive(Double speed) {
        if (speed > 0.0 && m_armMotor.getCurrentPosition() <= 1900) {
            m_armMotor.set(speed);
        } else if (speed < 0.0 && m_armMotor.getCurrentPosition() >= 10) {
            m_armMotor.set(speed);
        } else {
            m_armMotor.set(0.0);
        }
    }

    public void stopAll() { m_armMotor.set(0); }
}
