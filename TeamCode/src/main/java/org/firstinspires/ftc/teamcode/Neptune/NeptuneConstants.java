package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.dashboard.config.Config;

@Config
public class NeptuneConstants {
    public static  double NEPTUNE_INTAKE_SERVO_INITIAL_POS = 1.0;
    public static double NEPTUNE_INTAKE_SERVO_POS2 = 0.52;
    public static double MAX_SLIDE_MOTOR_POWER = 1.0;
    public static double MAX_VBAR_MOTOR_POWER = 0.8;
    public static double NEPTUNE_INTAKE_MOTOR_INTAKE_POWER = -1.0;
    public static double NEPTUNE_INTAKE_MOTOR_EJECT_POWER = 1.0;
    public static int NEPTUNE_SLIDE_POS1 = 800;
    public static int NEPTUNE_SLIDE_POS2 = 1400;
    public static double NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT_P = 0.002;
    public static double NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT_I = 0.0;
    public static double NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT_D = 0.0;
    public static int NEPTUNE_SLIDE_VBAR_POS_TOLERANCE = 10;
    public static double NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT = .02;
    public static int NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE = 150;
    public static int NEPTUNE_VBAR_MOTOR_TARGET_POSITION_UP = 450;
    public static int NEPTUNE_VBAR_MOTOR_TARGET_POSITION_DOWN = 0;
    public static double NEPTUNE_HANG_MOTOR_POWER = 0.95;
    public static double SLIDE_kP = 3;
    public static double SLIDE_kI = 0.00;
    public static double SLIDE_kD = 0.8;
    public static double SLIDE_kF = 0;
    public static double OUTAKE_OPEN_POSITION = 0.5;
    public static double OUTAKE_CLOSED_POSITION = 0.0;
    public static double AUTOOUTAKE_OPEN_POSITION = 0.4;
    public static double AUTOOUTAKE_CLOSED_POSITION = 0.75;
}
