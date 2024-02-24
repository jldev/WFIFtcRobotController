package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.dashboard.config.Config;

@Config
public class NeptuneConstants {
    public static double SLIDE_MOTOR_MANUAL_POWER = 0.3;
    public static double TRIGGER_THRESHOLD = 0.5;
    public static int CAMERA_EXPOSURE_TIME_MS = 40;
    public static int CAMERA_GAIN = 1;
    public static  double NEPTUNE_INTAKE_SERVO_INITIAL_POS = 1.0;
    public static double NEPTUNE_INTAKE_SERVO_POS2 = 0.52;
    public static double MAX_SLIDE_MOTOR_POWER = 1.0;
    public static double MAX_VBAR_MOTOR_POWER = 0.8;
    public static double NEPTUNE_INTAKE_MOTOR_INTAKE_POWER = -1.0;
    public static double NEPTUNE_INTAKE_MOTOR_EJECT_POWER = 1.0;
    public static int MIN_SAFE_POSTITION_FOR_VBAR = 800;
    public static int NEPTUNE_SLIDE_HOME = 0;
    public static int NEPTUNE_SLIDE_POS1 = 0;
    public static int NEPTUNE_SLIDE_POS2 = 900;
    public static int NEPTUNE_SLIDE_POS3 = 1400;
    public static double NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT = .02;
    public static int NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE = 50;
    public static double NEPTUNE_VBAR_TARGET_POSITION_UP = 0.40;
    public static double NEPTUNE_VBAR_TARGET_POSITION_DOWN = 1.0;
    public static double NEPTUNE_HANG_REST_POS= 1.0;
    public static double NEPTUNE_HANG_POS= 0.65;

    public static double NEPTUNE_WANTED_DISTANCE_FROM_BACKDROP = 6;

    public static double NEPTUNE_SECONDHANG_POS = 0.8;
    public static double NEPTUNE_HANG_MOTOR_UP_POWER = 1;
    public static double NEPTUNE_HANG_MOTOR_DOWN_POWER = -1;
    public static double SLIDE_kP = 3;
    public static double SLIDE_kI = 0.00;
    public static double SLIDE_kD = 0.8;
    public static double SLIDE_kF = 0;
    public static double OUTAKE_OPEN_POSITION = 0.5;
    public static double OUTAKE_CLOSED_POSITION = 0.0;
    public static double AUTOOUTAKE_OPEN_POSITION = 0.4;
    public static double AUTOOUTAKE_CLOSED_POSITION = 0.75;

    public static double DRONE_LAUNCHED = 0.5;

    public static double DRONE_NEUTRAL = 0;

    public static int SLIDE_HARD_STOP = 1500; //this stops the slides from going too high
}
