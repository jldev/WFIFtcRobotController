package org.firstinspires.ftc.teamcode.Helix;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HelixConstants {


    public static double SLIDE_SPEED = 1.0;

    public static double SLIDES_PID_TOLERANCE = 50;
    public static double SLIDES_PID_POS_COEFFICIENT = .85;
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
