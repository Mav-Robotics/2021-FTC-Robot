package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.arcrobotics.ftclib.geometry.Vector2d;

@Autonomous(name="BasicAuto", group="Autonomous")
public class BasicAuto extends LinearOpMode {

    static final String DRIVE_MODE = "RC";
    static final Boolean INTAKE_ENABLED = true;
    static final Boolean ARM_ENABLED = true;
    static final Boolean CAROUSEL_ENABLED = true;
    static final Boolean SENSORS_ENABLED = false;

    DrivetrainMecanum m_drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(true);

        // Drive Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312);



        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
        m_gyro.reset();


        // Drivetrain Subsystem
        m_drivetrain = new DrivetrainMecanum(motorBackLeft, motorBackRight,
                                                                 motorFrontLeft, motorFrontRight,
                                                                 telemetry, m_gyro, DRIVE_MODE);


        if (SENSORS_ENABLED) {
            Vision m_vision = new Vision(hardwareMap, telemetry);

            ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

            Sensors m_sensors = new Sensors(colorSensor, touchSensor, telemetry);
        }

        if (INTAKE_ENABLED) {
            /* Intake subsystem

            Enabled/disabled at the top setting the INTAKE_ENABLED value to true/false

            Left and right servos are set to continuous rotation.

            Holding the Y button runs them "forward" (out) using the IntakeOut command
            Holding the A button runs them "inward" using the IntakeIn command
            Letting go of the button stops the servos
             */
            CRServo servoIntake = new CRServo(hardwareMap, "servoIntake");

            Intake m_intake = new Intake(servoIntake, telemetry);

        }

        if (ARM_ENABLED) {
            /* Arm subsystem

            Enabled/disabled at the top setting the ARM_ENABLED value to true/false

            Left and right servos are set to continuous rotation.

            Holding the DPAD_UP drives the arm to the setpoint on the encoder using a PID
            Holding the DPAD_DOWN drives the arm to the setpoint on the encoder using a PID
            Holding the DPAD_LEFT drives the arm backwards at a set speed using the ArmDriveBackward
            Holding the DPAD_RIGHT drives the arm forward at a set speed using the ArmDriveForward
             */

            MotorEx motorArm = new MotorEx(hardwareMap, "motorArm", Motor.GoBILDA.RPM_312);
            motorArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            motorArm.resetEncoder();

            Arm m_arm = new Arm(motorArm, telemetry);

        }

        if (CAROUSEL_ENABLED) {
            /* Carousel subsystem

            Enabled/disabled at the top setting the ARM_ENABLED value to true/false


            Holding the DPAD_LEFT drives the arm backwards at a set speed using the CarouselDriveBackward
            Holding the DPAD_RIGHT drives the arm forward at a set speed using the CarouselDriveForward
             */

            MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");

            Carousel m_carousel = new Carousel(motorCarousel, telemetry);
    }

        telemetry.addLine("Robot Initialized");

        waitForStart();
        driveWithVector(new Vector2d(12, 3));
        sleep(1000);
        driveWithVector(new Vector2d(0, 0));

    }

    private void driveWithVector(Vector2d vector) {
        double[] speeds = normalize(new double[]{vector.getX(), vector.getY()});
        m_drivetrain.drive(speeds[0], speeds[1], 0);
    }

    /**
     * Normalize the wheel speeds if any value is greater than 1
     */
    private double[] normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }

        return wheelSpeeds;
    }
}
