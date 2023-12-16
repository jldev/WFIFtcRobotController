package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slides extends SubsystemBase {
    public enum LiftableIntakePosition {
        POSITION_1,
        POSITION_2,
    }

    ;
    private final MotorEx mSlideMotor;
    private final MotorEx mFourBarMotor;


    public Slides(MotorEx slidemotor, MotorEx fourbarmotor) {
        mSlideMotor = slidemotor;
        mFourBarMotor = fourbarmotor;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mFourBarMotor.setRunMode(MotorEx.RunMode.PositionControl);

        resetPositionCounter();
    }


    public void moveToPosition(int position) {
        mSlideMotor.setTargetPosition(position);
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.set(position);
    }


    public void resetPositionCounter() {
        mSlideMotor.resetEncoder();
    }
}
