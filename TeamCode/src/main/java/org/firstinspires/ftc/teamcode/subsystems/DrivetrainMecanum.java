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
    BNO055IMU m_imu;
    MotorEx m_motorFrontLeft, m_motorFrontRight, m_motorBackLeft, m_motorBackRight;

    static final Double STRAFE_MULT = 1.0;
    static final Double FORWARD_MULT = 0.5;
    static final Double TURN_MULT = 0.5;
    static final Double WHEEL_DIAMETER = 6.0;

    static final Double MM_PER_REV = Math.PI * WHEEL_DIAMETER;



    public DrivetrainMecanum(MotorEx motorBackLeft, MotorEx motorBackRight,
                             MotorEx motorFrontLeft, MotorEx motorFrontRight,
                             Telemetry telemetry, RevIMU gyro, String drivemode) {

        m_gyro = gyro;
        m_telemetry = telemetry;
        m_drivemode = drivemode;
        m_imu = m_gyro.getRevIMU();
        m_motorFrontLeft = motorFrontLeft;
        m_motorFrontRight = motorFrontRight;
        m_motorBackLeft = motorBackLeft;
        m_motorBackRight = motorBackRight;

        m_motorFrontLeft.setDistancePerPulse(MM_PER_REV);
        m_motorFrontRight.setDistancePerPulse(MM_PER_REV);
        m_motorBackLeft.setDistancePerPulse(MM_PER_REV);
        m_motorBackRight.setDistancePerPulse(MM_PER_REV);

        m_drivetrain = new MecanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        m_motorFrontLeft.resetEncoder();
        m_motorBackLeft.resetEncoder();
        m_motorBackRight.resetEncoder();
        m_motorFrontRight.resetEncoder();

        m_telemetry.addLine("Drivetrain Initialized");

    }


    @Override
    public void periodic() {
        m_telemetry.addData("Acceleration", m_imu.getAcceleration());
        m_telemetry.addData("Heading", m_gyro.getHeading());
        m_telemetry.addData("Motor Speeds","backLeft: %.2f, backRight: %.2f, frontLeft: %.2f, frontRight: %.2f",
                            m_motorBackLeft.get(), m_motorBackRight.get(), m_motorFrontLeft.get(), m_motorFrontRight.get());
        m_telemetry.addData("Encoder Values","backLeft: %d, backRight: %d, frontLeft: %d, frontRight: %d",
                            m_motorBackLeft.getCurrentPosition(), m_motorBackRight.getCurrentPosition(), m_motorFrontLeft.getCurrentPosition(), m_motorFrontRight.getCurrentPosition());
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

}
