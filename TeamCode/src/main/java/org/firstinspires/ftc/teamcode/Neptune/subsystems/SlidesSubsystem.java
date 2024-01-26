package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.opencv.dnn.Net;

public class SlidesSubsystem extends SubsystemBase {
    private int mSlideMotorTargetPosition = 0;
    private int mVBarMotorTargetPosition = 0;
    private int mSlidePositionOffset = 0;

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
    private final PIDMotor mVBarMotor;

    public SlidesSubsystem(MotorEx slideMotor, PIDMotor vBarMotor) {
        mSlideMotor = slideMotor;
        mVBarMotor = vBarMotor;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setPositionCoefficient(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT);
        mSlideMotor.setPositionTolerance(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE);
        mSlideMotor.setTargetPosition(mSlideMotorTargetPosition);
        mSlideMotor.resetEncoder();
        mSlidePosition = SlidesPosition.HOME_POS;
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor.encoder.setDirection(Motor.Direction.REVERSE);
        mVBarMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mVBarMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        mVBarMotor.setPositionCoefficient(NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT);
        mVBarMotor.setPIDCoefficients(NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT_P,
                NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT_I,
                NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_COEFFICIENT_D);
        mVBarMotor.setPositionTolerance(NeptuneConstants.NEPTUNE_SLIDE_VBAR_POS_TOLERANCE);
        mVBarMotor.resetEncoder();
        mVBarMotor.setTargetPosition(mVBarMotorTargetPosition);

        mVBarPosition = VBarPosition.DOWN;

    }

    @Override
    public void periodic(){

        if(!mSlideMotor.atTargetPosition()){
            mSlideMotor.set(NeptuneConstants.MAX_SLIDE_MOTOR_POWER);
        } else {
            mSlideMotor.set(0);
            switch (mSlidePosition) {
                case POSITION_1:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS1 + mSlidePositionOffset;
                    break;
                case POSITION_2:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS2 + mSlidePositionOffset;
                    break;
                case HOME_POS:
                    // home we have to wait for vbar to come down
                    if (mVBarMotor.atTargetPosition() && mVBarPosition == VBarPosition.DOWN) {
                        mSlideMotorTargetPosition = 0;
                      }
                    break;
            }
            mSlideMotor.setTargetPosition(mSlideMotorTargetPosition + mSlidePositionOffset);
        }

        if(!mVBarMotor.atTargetPosition()) {
             mVBarMotor.set(NeptuneConstants.MAX_VBAR_MOTOR_POWER);
        } else {
            mVBarMotor.set(0);
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
            mVBarMotor.setTargetPosition(mVBarMotorTargetPosition);
        }
    }

    public void moveToPosition(SlidesPosition position){
        if (position == SlidesPosition.POSITION_1){
            mVBarPosition = VBarPosition.UP;
            mSlidePosition = SlidesPosition.POSITION_1;
        }
        if(position == SlidesPosition.HOME_POS){
            mVBarPosition = VBarPosition.DOWN;
            mSlidePosition = SlidesPosition.HOME_POS;
            mVBarMotor.setTargetPosition(NeptuneConstants.NEPTUNE_VBAR_MOTOR_TARGET_POSITION_DOWN);
        }
        mSlidePositionOffset = 0;
    }

    public void UpdateOffset(int by)
    {
        mSlidePositionOffset += by * NeptuneConstants.NEPTUNE_SLIDE_OFFSET_CHANGE_BY;
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

    public void addTelemetry(Telemetry telemetry){
        telemetry.addLine(String.format("slide_setting - %s", mSlidePosition.toString()));
        telemetry.addLine(String.format("current_position - %d", mSlideMotor.getCurrentPosition()));
        telemetry.addLine(String.format("current_power %.2f", mSlideMotor.motor.getPower()));
        telemetry.addLine(String.format("target_position %d", mSlideMotorTargetPosition));

        telemetry.addLine(String.format("vbar_setting - %s", mVBarPosition.toString()));
        telemetry.addLine(String.format("vbarcurrentpos - %d", mVBarMotor.getCurrentPosition()));
        telemetry.addLine(String.format("vbarcurrentpwr %.2f", mVBarMotor.motor.getPower()));
        telemetry.addLine(String.format("vbarcurrentpos %d", mVBarMotorTargetPosition));
    }
}
