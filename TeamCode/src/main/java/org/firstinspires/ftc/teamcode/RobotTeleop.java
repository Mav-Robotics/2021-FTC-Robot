package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.autos.scoreFromStart.HighScore;
import org.firstinspires.ftc.teamcode.autos.scoreFromStart.LowScore;
import org.firstinspires.ftc.teamcode.autos.scoreFromStart.MidScore;
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

    static final String DRIVE_MODE = "RC";
    static final Boolean INTAKE_ENABLED = true;
    static final Boolean ARM_ENABLED = true;
    static final Boolean CAROUSEL_ENABLED = true;
    static final Boolean SENSORS_ENABLED = false;
    static final Integer PICKUP = 0;
    static final Integer LOW_TARGET = 585;
    static final Integer MID_TARGET = 1100;
    static final Integer HI_TARGET = 1800;

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

        //Gamepad
        GamepadEx m_driverGamepad = new GamepadEx(gamepad1);
        GamepadEx m_operatorGamepad = new GamepadEx(gamepad2);



        // Drivetrain Subsystem
        DrivetrainMecanum m_defaultdrive = new DrivetrainMecanum(motorBackLeft, motorBackRight,
                                                                 motorFrontLeft, motorFrontRight,
                                                                 telemetry, m_gyro, DRIVE_MODE);


        /* Default Drive Command
           Set the default command for the drivetrain to be gamepad controlled
        */



        register(m_defaultdrive);
        m_defaultdrive.setDefaultCommand(new DefaultDrive(m_defaultdrive, m_driverGamepad, telemetry));

        GamepadButton driver_x = m_driverGamepad.getGamepadButton(GamepadKeys.Button.X);
        GamepadButton driver_b = m_driverGamepad.getGamepadButton(GamepadKeys.Button.B);
        GamepadButton driver_y = m_driverGamepad.getGamepadButton(GamepadKeys.Button.Y);

        if (SENSORS_ENABLED) {
            Vision m_vision = new Vision(hardwareMap, telemetry);
            register(m_vision);

            ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");

            Sensors m_sensors = new Sensors(colorSensor, touchSensor, telemetry);
        }

        /* Intake subsystem

        Enabled/disabled at the top setting the INTAKE_ENABLED value to true/false

        Left and right servos are set to continuous rotation.

        Holding the Y button runs them "forward" (out) using the IntakeOut command
        Holding the A button runs them "inward" using the IntakeIn command
        Letting go of the button stops the servos
         */
        CRServo servoIntake = new CRServo(hardwareMap, "servoIntake");

        Intake m_intake = new Intake(servoIntake, telemetry);

        GamepadButton oper_a = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.A);
        GamepadButton oper_y = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.Y);

        oper_a.whileHeld(new IntakeIn(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake());
        oper_y.whileHeld(new IntakeOut(m_intake, telemetry)).whenReleased(() -> m_intake.stopIntake());


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

//            register(m_arm);
//            m_arm.setDefaultCommand(new ArmDefaultDrive(m_arm, m_operatorGamepad, telemetry));

        GamepadButton oper_dpad_up = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        GamepadButton oper_dpad_left = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        GamepadButton oper_dpad_right = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton oper_dpad_down = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);


        oper_dpad_up.whenPressed(new ArmToPosition(m_arm, HI_TARGET, telemetry));
        oper_dpad_right.whenPressed(new ArmToPosition(m_arm, MID_TARGET, telemetry));
        oper_dpad_left.whenPressed(new ArmToPosition(m_arm, LOW_TARGET, telemetry));
        oper_dpad_down.whenPressed(new ArmToPosition(m_arm, PICKUP, telemetry));


        /* Carousel subsystem

        Enabled/disabled at the top setting the ARM_ENABLED value to true/false


        Holding the DPAD_LEFT drives the arm backwards at a set speed using the CarouselDriveBackward
        Holding the DPAD_RIGHT drives the arm forward at a set speed using the CarouselDriveForward
         */

        MotorEx motorCarousel = new MotorEx(hardwareMap, "motorCarousel");

        Carousel m_carousel = new Carousel(motorCarousel, telemetry);

        GamepadButton oper_X = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.X);
        GamepadButton oper_B = m_operatorGamepad.getGamepadButton(GamepadKeys.Button.B);

        oper_X.whileHeld(new CarouselDriveBackward(m_carousel, telemetry)).whenReleased(() -> m_carousel.stopAll());
        oper_B.whileHeld(new CarouselDriveForward(m_carousel, telemetry)).whenReleased(() -> m_carousel.stopAll());



        driver_x.whenPressed(new LowScore(m_defaultdrive, m_arm, m_intake, telemetry));
        driver_b.whenPressed(new MidScore(m_defaultdrive, m_arm, m_intake, telemetry));
        driver_y.whenPressed(new HighScore(m_defaultdrive, m_arm, m_intake, telemetry));



        telemetry.addLine("Robot Initialized");
        telemetry.update();

    }
}
