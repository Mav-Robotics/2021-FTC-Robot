package org.firstinspires.ftc.teamcode.autos.opmodes.blue;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.autos.scoreAndPark.blue.BlueScoreAndParkWarehouse;


@Autonomous(name="Blue Score High Ware House and Park", group="Blue Score and Park")
public class AutoBlueScoreHighWareHouseAndPark extends CommandOpMode {

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

        if (RobotMap.SENSORS_ENABLED) {
            Vision m_vision = new Vision(hardwareMap, telemetry);
            register(m_vision);

            ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

            Sensors m_sensors = new Sensors(colorSensor, touchSensor, telemetry);
        }

        CRServo servoIntake = new CRServo(hardwareMap, "servoIntake");

        Intake m_intake = new Intake(servoIntake, telemetry);

        MotorEx motorArm = new MotorEx(hardwareMap, "motorArm", Motor.GoBILDA.RPM_312);
        motorArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorArm.resetEncoder();

        Arm m_arm = new Arm(motorArm, telemetry);

        MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");

        Carousel m_carousel = new Carousel(motorCarousel, telemetry);

        schedule(new BlueScoreAndParkWarehouse(m_defaultdrive, m_arm, m_intake, telemetry));


        telemetry.addLine("Robot Initialized");
        telemetry.update();

    }
}
