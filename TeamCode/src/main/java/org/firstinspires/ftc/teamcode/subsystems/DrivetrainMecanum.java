package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

        m_telemetry.addData("Heading", m_gyro.getHeading());
        m_telemetry.addData("Motor Speeds","backLeft: %.2f, backRight: %.2f, frontLeft: %.2f, frontRight: %.2f",
                            m_motorBackLeft.get(), m_motorBackRight.get(), m_motorFrontLeft.get(), m_motorFrontRight.get());
        m_telemetry.addData("Encoder Values","backLeft: %d, backRight: %d, frontLeft: %d, frontRight: %d",
                            m_motorBackLeft.getCurrentPosition(), m_motorBackRight.getCurrentPosition(), m_motorFrontLeft.getCurrentPosition(), m_motorFrontRight.getCurrentPosition());


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

    public Double getAverageEncoderValue() {
        return (m_motorBackRight.getCurrentPosition() +
                m_motorFrontRight.getCurrentPosition() +
                Math.abs(m_motorBackLeft.getCurrentPosition()) +
                Math.abs(m_motorFrontLeft.getCurrentPosition())) / 4.0;
    }

    public Double getAverageDistance() {
        return (m_motorBackRight.getDistance() +
                m_motorFrontRight.getDistance() +
                Math.abs(m_motorBackLeft.getDistance()) +
                Math.abs(m_motorFrontLeft.getDistance())) / 4.0 / 25.4;
    }

    public void resetEncoders() {
        m_motorFrontLeft.resetEncoder();
        m_motorBackLeft.resetEncoder();
        m_motorBackRight.resetEncoder();
        m_motorFrontRight.resetEncoder();
    }

}
