package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.dashboard.config.Config;

@Config
public class NeptuneConstants {
    public static double MAX_SLIDE_MOTOR_POWER = 0.75;
    public static double NEPTUNE_INTAKE_MOTOR_INTAKE_POWER = 0.5;
    public static double NEPTUNE_INTAKE_MOTOR_EJECT_POWER = -0.5;
    public static double NEPTUNE_INTAKE_SERVO_POS1 = 0.0;
    public static double NEPTUNE_INTAKE_SERVO_POS2 = 1.0;
    public static int NEPTUNE_SLIDE_POS1 = 1400;
    public static int NEPTUNE_SLIDE_POS2 = 2800;
    public static double NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT = 0.001;
    public static int NEPTUNE_SLIDE_VBAR_POS_TOLERANCE = 20;
    public static double NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT = 0.01;
    public static int NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE = 5;
    public static int NEPTUNE_VBAR_MOTOR_TARGET_POSITION_UP = 500;
    public static int NEPTUNE_VBAR_MOTOR_TARGET_POSITION_DOWN = 0;
    public static double NEPTUNE_HANG_MOTOR_POWER = 0.5;
    public static double SLIDE_kP = 3;
    public static double SLIDE_kI = 0.00;
    public static double SLIDE_kD = 0.8;
    public static double SLIDE_kF = 0;
}
