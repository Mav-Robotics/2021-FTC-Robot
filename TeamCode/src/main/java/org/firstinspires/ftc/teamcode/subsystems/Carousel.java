package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel extends SubsystemBase {
    Telemetry m_telemetry;
    MotorEx m_carouselMotor;

    public Carousel(MotorEx carouselMotor, Telemetry telemetry) {

        m_carouselMotor = carouselMotor;
        m_telemetry = telemetry;
        m_telemetry.addLine("Carousel Initialized");

    }


    @Override
    public void periodic() {

    }

    public void drive(Double speed) {
        m_carouselMotor.set(speed);
    }

    public void stopAll() { m_carouselMotor.set(0); }

    public void withTimeout(int i) {
    }

}
