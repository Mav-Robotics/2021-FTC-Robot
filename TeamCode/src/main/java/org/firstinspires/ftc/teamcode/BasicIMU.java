package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class BasicIMU extends LinearOpMode {
    private Gyroscope imu;

    @Override
    public void runOpMode(){
        imu = hardwareMap.get(Gyroscope.class, "imu");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Angular Velocity", imu.getAngularVelocity(AngleUnit.DEGREES));
            telemetry.update();

        }
    }

}
