package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.Utils.*;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

public class Sensors extends SubsystemBase {
    Telemetry m_telemetry;
    DistanceSensor m_distanceLeft, m_distanceRight, m_distanceRear;


    public Sensors(DistanceSensor distanceLeft, DistanceSensor distanceRight,
                   DistanceSensor distanceRear, Telemetry telemetry) {

        m_telemetry = telemetry;
        m_distanceLeft = distanceLeft;
        m_distanceRight = distanceRight;
        m_distanceRear = distanceRear;


        m_telemetry.addLine("Sensors Initialized");

    }


    @Override
    public void periodic() {
        m_telemetry.addData("distanceLeft", getLeftDistance());
        m_telemetry.addData("distanceRight", getRightDistance());
        m_telemetry.addData("distanceRear", getRearDistance());
    }

    public double getLeftDistance() {
        return Utils.round(m_distanceLeft.getDistance(DistanceUnit.INCH), 2);
    }

    public double getRightDistance() {
        return  Utils.round(m_distanceRight.getDistance(DistanceUnit.INCH), 2);
    }

    public double getRearDistance() {
        return  Utils.round(m_distanceRear.getDistance(DistanceUnit.INCH),2);
    }

}
