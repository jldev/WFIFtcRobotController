package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.opencv.dnn.Net;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class SlidesSubsystem extends SubsystemBase {
    private final Trigger encoderStopTrigger;
    private int mSlideMotorTargetPosition = 0;
    private int mVBarMotorTargetPosition = 0;
    private int mSlidePositionOffset = 0;

    private CommandOpMode mOpMode;

    public enum SlideSubsystemState {
        AUTO,
        MANUAL
    }

    public enum ManualControlDirection{
        UP,
        DOWN,
        OFF
    }
    public enum SlidesPosition {
        POSITION_1,
        POSITION_2,
        HOME_POS,
    }

    public enum VBarPosition {
        UP,
        DOWN
    }

    SlideSubsystemState mState;
    SlidesPosition mSlidePosition;
    ;
    VBarPosition mVBarPosition;

    private final MotorEx mSlideMotor;
    private final Servo mVbarServo;

    public SlidesSubsystem(MotorEx slideMotor, Servo VbarServo, CommandOpMode opmode) {
        mSlideMotor = slideMotor;
        mVbarServo = VbarServo;
        mVbarServo.setPosition(NeptuneConstants.NEPTUNE_OUTAKE_TARGET_POSITION_DOWN);
        mOpMode = opmode;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setPositionCoefficient(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT);
        mSlideMotor.setPositionTolerance(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE);
        mSlideMotor.setTargetPosition(mSlideMotorTargetPosition);
        mSlideMotor.resetEncoder();
        mSlidePosition = SlidesPosition.HOME_POS;
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor.encoder.setDirection(Motor.Direction.REVERSE);
        mVBarPosition = VBarPosition.DOWN;
        mState = SlideSubsystemState.AUTO;

        BooleanSupplier encoderStopSupplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return mSlideMotor.getCurrentPosition() > NeptuneConstants.SLIDE_HARD_STOP;
            }
        };

        encoderStopTrigger = new Trigger(encoderStopSupplier);
        encoderStopTrigger.whenActive(() -> {
           this.manualSlideControl(ManualControlDirection.OFF);
        });

    }

    public void autoState(){
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
                    if (mVBarPosition == VBarPosition.DOWN) {
                        mSlideMotorTargetPosition = 0;
                      }
                    break;
            }
            mSlideMotor.setTargetPosition(mSlideMotorTargetPosition + mSlidePositionOffset);
        }

        switch (mVBarPosition) {
            case UP:
                //up we have to wait for the slides to extend
                if (mSlideMotor.atTargetPosition() && mSlidePosition != SlidesPosition.HOME_POS) {
                    mVbarServo.setPosition(NeptuneConstants.NEPTUNE_OUTAKE_TARGET_POSITION_UP);
                }
                break;
            case DOWN:
                mVbarServo.setPosition(NeptuneConstants.NEPTUNE_OUTAKE_TARGET_POSITION_DOWN);
                break;
        }
    }

    public void manualState(){

    }

    @Override
    public void periodic(){
        switch (mState){
            case AUTO:
                autoState();
                break;
            case MANUAL:
                manualState();
                break;
        }
    }

    private void changeSlideState(SlideSubsystemState newState){
        if (mState != newState){ //we need to change the state
            if (newState == SlideSubsystemState.AUTO){
                mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
            } else {
                //we are changing to MANUAL
                mSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
            }
        }
        mState = newState;
    }
    public void moveToPosition(SlidesPosition position){
        // anytime the user wants to move to a position we need to be in auto state
        changeSlideState(SlideSubsystemState.AUTO);
        if (position == SlidesPosition.POSITION_1){
            mVBarPosition = VBarPosition.UP;
            mSlidePosition = SlidesPosition.POSITION_1;
        }
        if(position == SlidesPosition.HOME_POS){
            mVBarPosition = VBarPosition.DOWN;
            mSlidePosition = SlidesPosition.HOME_POS;
            mVbarServo.setPosition(NeptuneConstants.NEPTUNE_OUTAKE_TARGET_POSITION_DOWN);
            mOpMode.sleep(1000);

        }
        mSlidePositionOffset = 0;
    }

    public void stopMotorResetEncoder() {
        mSlideMotor.encoder.reset();
        mSlidePosition = SlidesPosition.HOME_POS;
    }

    public void manualSlideControl(ManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        switch (direction){
            case UP:
                mSlideMotor.set(NeptuneConstants.SLIDE_MOTOR_MANUAL_POWER);
                break;
            case DOWN:
                mSlideMotor.set(-NeptuneConstants.SLIDE_MOTOR_MANUAL_POWER);
                break;
            case OFF:
                mSlideMotor.set(0);
                break;
        }

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
        telemetry.addLine(String.format("vbarcurrentpwr %.2f", mVbarServo.getPosition()));
        telemetry.addLine(String.format("vbarcurrentpos %d", mVBarMotorTargetPosition));
    }
}
