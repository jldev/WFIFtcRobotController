package org.firstinspires.ftc.teamcode.Helix;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HelixConstants {



    public static double SLIDE_SPEED = 1.0;
    public static int SLIDE_MANUAL_SPEED = 25;
    public static double PIVOT_SPEED = 1.0;
    public static int PIVOT_MANUAL_SPEED = 15;
    public static double SLIDE_LOCK_POWER = 0.2;

    public static double SLIDES_PID_TOLERANCE = 10;
    public static double SLIDES_PID_POS_COEFFICIENT = .25;
    public static double HORIZONTAL_PID_P = 0.0285;
    public static double HORIZONTAL_PID_I = 0.001;
    public static double HORIZONTAL_PID_D = 0.001;
    public static double HORIZONTAL_PID_F = 0.0;
    public static double VERTICAL_PID_P = 0.05;
    public static double VERTICAL_PID_I = 0.0;
    public static double VERTICAL_PID_D = 0.00;
    public static double VERTICAL_PID_F = 0.0;
    public static double PIVOT_PID_P = 0.05;
    public static double PIVOT_PID_I = 0.0;
    public static double PIVOT_PID_D = 0.00;
    public static double PIVOT_PID_F = 0.0;
    public  static double LIFT_POS_0 = 0.975f;
    public  static double LIFT_POS_1 = 0.9f;

    public static double LIFT_POS_2 = 0.5f;

    //    vertical positions

    public static int VERTICAL_SLIDE_HOME = 0;
    public static int VERTICAL_SLIDE_WALL = 1000;
    public static int VERTICAL_SLIDE_HANG = 0;
    public static int VERTICAL_SLIDE_BASKET = 3853;

    // horizontal positions
    public static int HORIZONTAL_SLIDE_HOME = 0;
    public static int HORIZONTAL_SLIDE_WALL = 0;
    public static int HORIZONTAL_SLIDE_HANG = 0;
    public static int HORIZONTAL_SLIDE_BASKET = 0;

    //    pivot positions

    public static int PIVOT_HOME = 50;
    public static int PIVOT_HANG = 300;
    public static int PIVOT_BASKET = 1100;
    public static int PIVOT_SUB = -25;

    //   claw positions

    public static double GRIPPER_CLOSED_VALUE = 0.85f;
    public static double GRIPPER_OPEN_VALUE = 0.2f;
    public static double CLAW_YAW_INIT = 0.0f;
    public static double CLAW_PITCH_INIT = 0.35f;
    
}
