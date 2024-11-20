package org.firstinspires.ftc.teamcode.Helix;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HelixConstants {


    public static double SLIDE_SPEED = 1.0;
    public static int SLIDE_MANUAL_SPEED = 25;
    public static double PIVOT_SPEED = 1.0;
    public static double SLIDE_LOCK_POWER = 0.2;

    public static double SLIDES_PID_TOLERANCE = 10;
    public static double SLIDES_PID_POS_COEFFICIENT = .25;
    public static double HORIZONTAL_PID_P = 0.025;
    public static double HORIZONTAL_PID_I = 0.0;
    public static double HORIZONTAL_PID_D = 0.0;
    public static double HORIZONTAL_PID_F = 0.0;
    public static double VERTICAL_PID_P = 0.4;
    public static double VERTICAL_PID_I = 0.8;
    public static double VERTICAL_PID_D = 0.01;
    public static double VERTICAL_PID_F = 0.0;
    public  static double LIFT_POS_0 = 0.975f;
    public  static double LIFT_POS_1 = 0.9f;

    public static double LIFT_POS_2 = 0.5f;

    public static double GRIPPER_CLOSED_VALUE = 0.2f;

    public static double GRIPPER_OPEN_VALUE = 0.5f;

    //    slide positions

    public static int SLIDE_HOME = 0;
    public static int SLIDE_WALL = 1000;
    public static int SLIDE_HANG = 1500;
    public static int SLIDE_BASKET = 3853;
    
}
