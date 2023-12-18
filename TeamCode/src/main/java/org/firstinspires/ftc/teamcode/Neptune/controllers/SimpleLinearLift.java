package org.firstinspires.ftc.teamcode.Neptune.controllers;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Direction;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class SimpleLinearLift {

    MotorEx mSlideMotor;

    public SimpleLinearLift(MotorEx liftMotor) {
        mSlideMotor = liftMotor;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);

        resetPositionCounter();
    }

    public void moveLift(double power) {
        mSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        mSlideMotor.set(power);
    }

    public void moveToPosition(int position) {
        mSlideMotor.setTargetPosition(position);
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.set(position);
    }

    public void moveWithTimer(int activeTime, Direction direction) {
        Timing.Timer timer = new Timing.Timer(activeTime, TimeUnit.MILLISECONDS);
        timer.start();

        int multiplier = direction == Direction.UP ? 1 : -1;

        while (!timer.done()) {
            mSlideMotor.set(multiplier * (activeTime - timer.elapsedTime()) / (double) activeTime);
        }
    }

    public void resetPositionCounter() {
        mSlideMotor.resetEncoder();
    }

    public int getPosition() {
       return mSlideMotor.getCurrentPosition();
    }

    public boolean atTargetPosition(){
        return mSlideMotor.atTargetPosition();
    }
}