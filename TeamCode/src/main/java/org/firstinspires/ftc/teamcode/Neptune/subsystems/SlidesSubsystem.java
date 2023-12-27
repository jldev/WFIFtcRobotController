package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class SlidesSubsystem extends SubsystemBase {
    private int mSlideMotorTargetPosition = 0;
    private int mVBarMotorTargetPosition = 0;

    public enum SlidesPosition {
        POSITION_1,
        POSITION_2,
        HOME_POS,
    }

    public enum VBarPosition {
        UP,
        DOWN
    }

    SlidesPosition mSlidePosition;
    ;
    VBarPosition mVBarPosition;

    private final MotorEx mSlideMotor;
    private final MotorEx mVBarMotor;


    public SlidesSubsystem(MotorEx slideMotor, MotorEx vBarMotor) {
        mSlideMotor = slideMotor;
        mVBarMotor = vBarMotor;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mVBarMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mVBarMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mSlideMotor.setPositionCoefficient(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT);
        mSlideMotor.setPositionTolerance(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE);
        mSlideMotor.resetEncoder();
        mSlidePosition = SlidesPosition.HOME_POS;

        mVBarMotor.setPositionCoefficient(NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT);
        mVBarMotor.setPositionTolerance(NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_TOLERANCE);
        mVBarMotor.resetEncoder();
        mVBarPosition = VBarPosition.DOWN;
    }

    @Override
    public void periodic(){
        if(mSlideMotor.atTargetPosition()) {
            switch (mSlidePosition) {
                case POSITION_1:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS1;
                    break;
                case POSITION_2:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS2;
                    break;
                case HOME_POS:
                    // home we have to wait for vbar to come down
                    if (mVBarMotor.atTargetPosition() && mVBarPosition == VBarPosition.DOWN){
                        mSlideMotorTargetPosition = 0;
                    }
                    break;
            }
            moveToPosition(mSlideMotor, mSlideMotorTargetPosition);
        } else {
            mSlideMotor.set(mSlideMotorTargetPosition);
        }
        if(mVBarMotor.atTargetPosition()) {
            switch (mVBarPosition) {
                case UP:
                    //up we have to wait for the slides to extend
                    if (mSlideMotor.atTargetPosition() && mSlidePosition != SlidesPosition.HOME_POS) {
                        mVBarMotorTargetPosition = NeptuneConstants.NEPTUNE_VBAR_MOTOR_TARGET_POSITION_UP;
                    }
                    break;
                case DOWN:
                    mVBarMotorTargetPosition = NeptuneConstants.NEPTUNE_VBAR_MOTOR_TARGET_POSITION_DOWN;
                    break;
            }
            moveToPosition(mVBarMotor, mVBarMotorTargetPosition);
        } else {
            mVBarMotor.set(mVBarMotorTargetPosition);
        }
    }

    private void moveToPosition(MotorEx motor, int position) {
        motor.setTargetPosition(position);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.set(position);
    }

    public void moveToPosition(SlidesPosition position){
        if (position == SlidesPosition.POSITION_1){
            mVBarPosition = VBarPosition.UP;
            mSlidePosition = SlidesPosition.POSITION_1;
        }
        if(position == SlidesPosition.HOME_POS){

            mVBarPosition = VBarPosition.DOWN;
            mSlidePosition = SlidesPosition.HOME_POS;
        }
    }

    public boolean isBusy (){
        return !mSlideMotor.atTargetPosition();
    }

    public int getSlidePosition(){
        return mSlideMotor.encoder.getPosition();
    }
    public void resetPositionCounter() {
        mSlideMotor.resetEncoder();
    }
}
