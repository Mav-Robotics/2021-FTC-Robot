package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils;

public class DrivetrainMecanum extends SubsystemBase {
    RevIMU m_gyro;
    Telemetry m_telemetry;
    MecanumDrive m_drivetrain;
    String m_drivemode;
    MotorEx m_motorFrontLeft, m_motorFrontRight, m_motorBackLeft, m_motorBackRight;

    static final Double STRAFE_MULT = 1.0;
    static final Double FORWARD_MULT = 0.5;
    static final Double TURN_MULT = 0.5;

    static final Double MM_PER_PULSE = 0.55;



    public DrivetrainMecanum(MotorEx motorBackLeft, MotorEx motorBackRight,
                             MotorEx motorFrontLeft, MotorEx motorFrontRight,
                             Telemetry telemetry, RevIMU gyro, String drivemode) {

        m_gyro = gyro;
        m_telemetry = telemetry;
        m_drivemode = drivemode;
        m_motorFrontLeft = motorFrontLeft;
        m_motorFrontRight = motorFrontRight;
        m_motorBackLeft = motorBackLeft;
        m_motorBackRight = motorBackRight;

        m_motorFrontLeft.setDistancePerPulse(MM_PER_PULSE);
        m_motorFrontRight.setDistancePerPulse(MM_PER_PULSE);
        m_motorBackLeft.setDistancePerPulse(MM_PER_PULSE);
        m_motorBackRight.setDistancePerPulse(MM_PER_PULSE);

        m_drivetrain = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
        resetEncoders();

        m_telemetry.addLine("Drivetrain Initialized");

    }


    @Override
    public void periodic() {

        m_telemetry.addData("Avg. Distance", getAverageDistance());
        m_telemetry.addData("Avg. Encoder Value", getAverageEncoderValue());
        m_telemetry.addData("Heading", getCurrentHeading());
        m_telemetry.addData("Absolute Heading", m_gyro.getAbsoluteHeading());
        m_telemetry.addData("Rotation", m_gyro.getRotation2d());
        m_telemetry.addData("Modulus error", Utils.getModulusError(0, getCurrentHeading(),-180.0, 180.0));


        m_telemetry.addData("Motor Speeds","backLeft: %.2f, backRight: %.2f, frontLeft: %.2f, frontRight: %.2f",
                            m_motorBackLeft.get(), m_motorBackRight.get(), m_motorFrontLeft.get(), m_motorFrontRight.get());
        m_telemetry.addData("Encoder Values","backLeft: %f, backRight: %f, frontLeft: %f, frontRight: %f",
                            m_motorBackLeft.getDistance(), m_motorBackRight.getDistance(), m_motorFrontLeft.getDistance(), m_motorFrontRight.getDistance());

        m_telemetry.update();
    }


    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        /* Takes input from the gamepad (in the command) and runs the motors
        If the mode is robot centric, the sticks will work as if you're on the robot
        facing forward all the time.

        If the mode is field centric, the sticks will always move the robot in the direction
        you point them. As if you were looking at the field from the top down
         */

        strafeSpeed = strafeSpeed * STRAFE_MULT;
        forwardSpeed = forwardSpeed * FORWARD_MULT;
        turnSpeed = turnSpeed * TURN_MULT;

        if (m_drivemode.equals("RC")) {
            m_drivetrain.driveRobotCentric(strafeSpeed,forwardSpeed,
                    turnSpeed);
        }
        else if (m_drivemode.equals("FC")) {
            m_drivetrain.driveFieldCentric(strafeSpeed,
                    forwardSpeed,
                    turnSpeed, m_gyro.getHeading());
        }
    }

    public Double getCurrentHeading() {
        return m_gyro.getAbsoluteHeading();
    }

    public void resetHeading() {
        m_gyro.reset();
    }


    public Double getAverageEncoderValue() {
        Integer multiplier;
        if (m_motorBackLeft.getCurrentPosition() < 0.0 && m_motorFrontLeft.getCurrentPosition() < 0.0) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }
        return (Math.abs(m_motorBackRight.getCurrentPosition()) +
                Math.abs(m_motorFrontRight.getCurrentPosition()) +
                Math.abs(m_motorBackLeft.getCurrentPosition()) +
                Math.abs(m_motorFrontLeft.getCurrentPosition())) / 4.0 * multiplier;
    }

    public Double getAverageDistance() {
        Integer multiplier;
        if (m_motorBackLeft.getDistance() < 0.0 && m_motorFrontLeft.getDistance() < 0.0) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }

        return (Math.abs(m_motorBackRight.getDistance()) +
                Math.abs(m_motorFrontRight.getDistance()) +
                Math.abs(m_motorBackLeft.getDistance()) +
                Math.abs(m_motorFrontLeft.getDistance())) / 4.0 / 25.4 * multiplier;
    }

    public void resetEncoders() {
        m_motorFrontLeft.resetEncoder();
        m_motorBackLeft.resetEncoder();
        m_motorBackRight.resetEncoder();
        m_motorFrontRight.resetEncoder();
    }

    public void stopAll(){
        m_motorFrontLeft.stopMotor();
        m_motorBackLeft.stopMotor();
        m_motorBackRight.stopMotor();
        m_motorFrontRight.stopMotor();
    }

}
