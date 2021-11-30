package org.firstinspires.ftc.teamcode;

public class RobotMap {

    // General settings
    public static final String DRIVE_MODE = "RC";
    public static final Boolean ARM_STICK_DRIVE = false;
    public static final Boolean SENSORS_ENABLED = false;

    // Arm speeds, heights and limits
    public static final Double ARM_MOTOR_SPEED = 1.0;
    public final static Integer ARM_HIGH_LIMIT = 1850;
    public final static Integer ARM_LOW_LIMIT = 0;
    public static final Integer PICKUP = 0;
    public static final Integer LOW_TARGET = 585;
    public static final Integer MID_TARGET = 1220;
    public static final Integer HI_TARGET = 1800;

    // Intake speeds
    public static final Double INTAKE_SPEED_IN = -1.0;
    public static final Double INTAKE_SPEED_OUT = 1.0;

    // Drivetrain
    public static final Double STRAFE_MULT = 1.0;
    public static final Double FORWARD_MULT = 0.5;
    public static final Double TURN_MULT = 0.5;
    public static final Double MM_PER_PULSE = 0.55;

    // Carousel
    public static final Double CAROUSEL_SPEED_FORWARD = 0.23;
    public static final Double CAROUSEL_SPEED_REVERSE = -0.23;

    // Vision
    public static final String VUFORIA_KEY = "ARKNcpL/////AAABmaul75WJu02hpEsBG/MnvsZ0aacsUMH0zc+d53A" +
            "KGDU3mzdXJQzSDPuea0rokovM0/U3INJNoaNvGx+Xnk9tFdgMVitg+hE32fMsH4f5KLF9CqJyqRynTBo55jfOsN4UbPMO6ij" +
            "MdpQg7PPUg8O7pd5HNOjoDx+MKgZ+FldA4uCCj5vEansKoP5++7V0a/E/0j4MClpdkUA/7cf9WSNS8opUnl9lAyNpOEiOa0b" +
            "q2KbzC5234XlaqzE7it5yl9QhstUyAfy1rRyRYc7ClclkuK1kleXepW2FQED5MsC3S+4buqtAe2pnJA7QyHJ3PGUBQd3L5PF" +
            "VVDeXRGHIF6ZKij3R6zKbWc6/NVSc2J7S5Uz2";

}
