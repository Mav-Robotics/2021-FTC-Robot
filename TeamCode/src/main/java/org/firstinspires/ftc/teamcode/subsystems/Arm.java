package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
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
    MotorEx m_armMotor;

    private final static Integer ARM_HIGH = 1850;
    private final static Integer ARM_LOW = 0;

    public Arm(MotorEx armMotor, Telemetry telemetry) {

        m_armMotor = armMotor;
        m_telemetry = telemetry;

        m_armMotor.setRunMode(Motor.RunMode.PositionControl);
        m_armMotor.setPositionTolerance(5.0);

        m_telemetry.addLine("Arm Initialized");


    }


    @Override
    public void periodic() {
        m_telemetry.addData("Arm Motor Clicks", m_armMotor.getCurrentPosition());
        m_telemetry.addData("Arm Output", m_armMotor.get());
    }



    public void driveToSetPoint(Integer setpoint) {
        if (setpoint >= ARM_LOW && setpoint <= ARM_HIGH) {
            m_armMotor.setTargetPosition(setpoint);
            while (!m_armMotor.atTargetPosition()) {
                m_armMotor.set(0.5);
            }
        } else {
            m_telemetry.addData("ERROR", "Setpoint is beyond min/max");
        }
        stopAll();
    }

    public void drive(Double speed) {
        if (speed > 0.0 && !armAtTop()) {
            m_armMotor.set(speed);
        } else if (speed < 0.0 && !armAtBottom()) {
            m_armMotor.set(speed);
        } else {
            m_armMotor.set(0.0);
        }
    }

    public boolean atSetPoint() { return m_armMotor.atTargetPosition(); }

    public Integer armPosition() {
        return m_armMotor.getCurrentPosition();
    }


    public boolean armAtTop() {
        return m_armMotor.getCurrentPosition() >= ARM_HIGH;
    }

    public boolean armAtBottom() {
        return m_armMotor.getCurrentPosition() <= ARM_LOW;
    }

    public void stopAll() { m_armMotor.set(0); }
}
