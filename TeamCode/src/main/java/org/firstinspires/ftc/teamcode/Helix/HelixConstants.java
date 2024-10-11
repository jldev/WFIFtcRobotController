package org.firstinspires.ftc.teamcode.Helix;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;

@Config
public class HelixConstants {


    public static double SLIDE_MOTOR_MANUAL_POWER = 0.25;

    public static double SLIDES_PID_TOLERANCE = 50;
    public static double SLIDES_PID_POS_COEFFICIENT = .5;
    public  static double LIFT_POS_0 = 0.975f;
    public  static double LIFT_POS_1 = 0.9f;

    public static double LIFT_POS_2 = 0.5f;

    public static double GRIPPER_CLOSED_VALUE = 0.2f;

    public static double GRIPPER_OPEN_VALUE = 0.5f;
    
}
