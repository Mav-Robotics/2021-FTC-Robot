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
import org.firstinspires.ftc.teamcode.RobotMap;

public class Arm extends SubsystemBase {
    Telemetry m_telemetry;
    MotorEx m_armMotor;

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
        if (setpoint >= RobotMap.ARM_LOW_LIMIT && setpoint <= RobotMap.ARM_HIGH_LIMIT) {
            m_armMotor.setTargetPosition(setpoint);
            while (!m_armMotor.atTargetPosition()) {
                m_armMotor.set(RobotMap.ARM_MOTOR_SPEED);
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
        return m_armMotor.getCurrentPosition() >= RobotMap.ARM_HIGH_LIMIT;
    }

    public boolean armAtBottom() {
        return m_armMotor.getCurrentPosition() <= RobotMap.ARM_LOW_LIMIT;
    }

    public void stopAll() { m_armMotor.set(0); }
}
