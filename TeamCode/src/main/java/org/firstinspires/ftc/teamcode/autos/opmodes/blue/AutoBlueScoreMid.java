package org.firstinspires.ftc.teamcode.autos.opmodes.blue;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.autos.scoreAndPark.red.RedMidScoreAndPark;
import org.firstinspires.ftc.teamcode.autos.scoreFromStart.blue.BlueMidScore;
import org.firstinspires.ftc.teamcode.autos.scoreFromStart.red.RedMidScore;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


@Autonomous(name="Blue Score Mid", group="Blue Score")
public class AutoBlueScoreMid extends CommandOpMode {

    @Override
    public void initialize() {

//        telemetry.setAutoClear(true);

        // Drive Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312);



        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
        m_gyro.init();
        m_gyro.reset();


        // Drivetrain Subsystem
        DrivetrainMecanum m_defaultdrive = new DrivetrainMecanum(motorBackLeft, motorBackRight,
                                                                 motorFrontLeft, motorFrontRight,
                                                                 telemetry, m_gyro, "RC");

        // Sensors
        Rev2mDistanceSensor m_distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        Rev2mDistanceSensor m_distanceRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
        Rev2mDistanceSensor m_distanceRear = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRear");

        Sensors m_sensors = new Sensors(m_distanceLeft, m_distanceRight, m_distanceRear, telemetry);



        CRServo servoIntake = new CRServo(hardwareMap, "servoIntake");

        Intake m_intake = new Intake(servoIntake, telemetry);

        MotorEx motorArm = new MotorEx(hardwareMap, "motorArm", Motor.GoBILDA.RPM_312);
        motorArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorArm.resetEncoder();

        Arm m_arm = new Arm(motorArm, telemetry);


        MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");

        Carousel m_carousel = new Carousel(motorCarousel, telemetry);

        schedule(new BlueMidScore(m_defaultdrive, m_arm, m_intake, telemetry));


        telemetry.addLine("Robot Initialized");
        telemetry.update();

    }
}
