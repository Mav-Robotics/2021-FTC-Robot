package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.teamcode.commands.ArmDefaultDrive;
import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveBackward;
import org.firstinspires.ftc.teamcode.commands.CarouselDriveForward;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.IntakeOut;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


@TeleOp(name="RobotTeleop", group="Competition")
public class RobotTeleop extends CommandOpMode {

    @Override
    public void initialize() {

        // Drive Motors
        MotorEx motorBackLeft = new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorBackRight = new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontLeft = new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_312);
        MotorEx motorFrontRight = new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_312);

        // Intake Motors
        CRServo servoIntake = new CRServo(hardwareMap, "servoIntake");

        // Arm Motors
        MotorEx motorArm = new MotorEx(hardwareMap, "motorArm", Motor.GoBILDA.RPM_312);
        motorArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorArm.resetEncoder();


        // Carousel Motor
        MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");


        //Gyro
        RevIMU m_gyro = new RevIMU(hardwareMap, "imu");
        m_gyro.init();
        m_gyro.reset();

        //Gamepad
        GamepadEx m_driverGamepad = new GamepadEx(gamepad1);
        GamepadEx m_operatorGamepad = new GamepadEx(gamepad2);

        // Sensors
        DistanceSensor m_distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        DistanceSensor m_distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        DistanceSensor m_distanceRear = hardwareMap.get(DistanceSensor.class, "distanceRear");

        Sensors m_sensors = new Sensors(m_distanceLeft, m_distanceRight, m_distanceRear, telemetry);


        // Subsystems

        Intake m_intake = new Intake(servoIntake, telemetry);
        Arm m_arm = new Arm(motorArm, telemetry);
        Carousel m_carousel = new Carousel(motorCarousel, telemetry);

        DrivetrainMecanum m_defaultdrive = new DrivetrainMecanum(motorBackLeft, motorBackRight,
                motorFrontLeft, motorFrontRight,
                telemetry, m_gyro, RobotMap.DRIVE_MODE);



        // Operator Buttons
        GamepadButton oper_a = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.A);
        GamepadButton oper_y = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.Y);
        GamepadButton oper_X = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.X);
        GamepadButton oper_B = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.B);

        GamepadButton oper_dpad_up = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        GamepadButton oper_dpad_left = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        GamepadButton oper_dpad_right = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton oper_dpad_down = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        // Driver buttons
        GamepadButton driver_a = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.A);
        GamepadButton driver_x = m_driverGamepad.getGamepadButton(GamepadKeys.Button.X);
        GamepadButton driver_b = m_driverGamepad.getGamepadButton(GamepadKeys.Button.B);
        GamepadButton driver_y = m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y);

        GamepadButton driver_dpad_up = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        GamepadButton driver_dpad_left = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        GamepadButton driver_dpad_right = m_driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton driver_dpad_down = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);


        // Map Operator Commands

        oper_a.whileHeld(new IntakeIn(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake()); // Intake In
        oper_y.whileHeld(new IntakeOut(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake()); // Intake Out
        oper_B.whileHeld(new CarouselDriveBackward(m_carousel, telemetry)).whenReleased(() -> m_carousel.stopAll()); // Carousel backward
        oper_X.whileHeld(new CarouselDriveForward(m_carousel, telemetry)).whenReleased(() -> m_carousel.stopAll()); // Carousel forward


//        // Map Driver Commands
//        driver_dpad_left.whenPressed(new StrafeDistance(m_defaultdrive, 0.5, 10.0, "LEFT", telemetry));
//        driver_dpad_right.whenPressed(new StrafeDistance(m_defaultdrive, 0.5, 10.0, "RIGHT", telemetry));
//
//
//        // Auto scoring commands (temporary)
//        driver_x.whenPressed(new LowScoreAndPark(m_defaultdrive, m_arm, m_intake, telemetry));
//        driver_b.whenPressed(new MidScoreAndPark(m_defaultdrive, m_arm, m_intake, telemetry));
//        driver_y.whenPressed(new HighScoreAndPark(m_defaultdrive, m_arm, m_intake, telemetry));


        // Default Commands

        register(m_defaultdrive);
        m_defaultdrive.setDefaultCommand(new DefaultDrive(m_defaultdrive, m_driverGamepad, telemetry));

        if (RobotMap.ARM_STICK_DRIVE) {
            register(m_arm);
            m_arm.setDefaultCommand(new ArmDefaultDrive(m_arm, m_operatorGamepad, telemetry));

        } else {
            oper_dpad_up.whenPressed(new ArmToPosition(m_arm, RobotMap.HI_TARGET, telemetry));
            oper_dpad_right.whenPressed(new ArmToPosition(m_arm, RobotMap.MID_TARGET, telemetry));
            oper_dpad_left.whenPressed(new ArmToPosition(m_arm, RobotMap.LOW_TARGET, telemetry));
            oper_dpad_down.whenPressed(new ArmToPosition(m_arm, RobotMap.PICKUP, telemetry));
        }

        telemetry.addLine("Robot Initialized");
        telemetry.update();

    }
}
