package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import static org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants.MAX_SLIDE_MOTOR_POWER;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

import java.util.function.BooleanSupplier;

public class SlidesSubsystem extends SubsystemBase {
    private final Trigger encoderStopTrigger;
    private final Neptune mNeptune;
    private int mSlideMotorTargetPosition = 0;
    private int mVBarMotorTargetPosition;

    private final CommandOpMode mOpMode;

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
        HOME_POS,
        POSITION_1,
        POSITION_2,
        POSITION_3,
    }

    public enum VBarPosition {
        UP,
        DOWN
    }

    SlideSubsystemState mState;
    SlidesPosition mSlidesCurrentPosition;
    VBarPosition mVBarCurrentPosition = VBarPosition.DOWN;
    VBarPosition mVBarNextPosition = VBarPosition.DOWN;

    private final MotorEx mSlideMotor;
    private final Servo mVbarServo;

    private final AnalogInput mVbarAnalog;

    public SlidesSubsystem(Neptune neptune, MotorEx slideMotor, Servo VbarServo, CommandOpMode opmode, AnalogInput vbarAnalog) {
        mNeptune = neptune;
        mSlideMotor = slideMotor;
        mVbarServo = VbarServo;
        mVbarServo.setPosition(NeptuneConstants.NEPTUNE_VBAR_TARGET_POSITION_DOWN);
        mVbarAnalog = vbarAnalog;
        mOpMode = opmode;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setPositionCoefficient(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_COEFFICIENT);
        mSlideMotor.setPositionTolerance(NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE);
        mSlideMotor.setTargetPosition(mSlideMotorTargetPosition);
        mSlideMotor.resetEncoder();
        mSlidesCurrentPosition = SlidesPosition.HOME_POS;
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor.encoder.setDirection(Motor.Direction.REVERSE);
        mVBarCurrentPosition = VBarPosition.DOWN;
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
            mSlideMotor.set(MAX_SLIDE_MOTOR_POWER);
        } else {
            mSlideMotor.set(0);
            switch (mSlidesCurrentPosition) {
                case POSITION_1:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS1;
                    mVBarNextPosition = VBarPosition.UP;
                    break;
                case POSITION_2:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS2;
                    mVBarNextPosition = VBarPosition.UP;
                    break;
                case POSITION_3:
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_POS3;
                    mVBarNextPosition = VBarPosition.UP;
                    break;
                case HOME_POS:
                    mVBarNextPosition = VBarPosition.DOWN;
                    mSlideMotorTargetPosition = NeptuneConstants.NEPTUNE_SLIDE_HOME;
                    break;
            }

        }

        // if the user has requested the vbar to move we have to be in safe spot to do so
        if(mVBarNextPosition != mVBarCurrentPosition){
            if (mSlideMotor.getCurrentPosition() < (NeptuneConstants.MIN_SAFE_POSTITION_FOR_VBAR - NeptuneConstants.NEPTUNE_SLIDE_MOTOR_POS_TOLERANCE)) {
                mSlideMotorTargetPosition = NeptuneConstants.MIN_SAFE_POSTITION_FOR_VBAR;
            } else {
                // we are good the slides are high enough to change the vbar position
                mVBarCurrentPosition = mVBarNextPosition;
                // set the position to the safe position
                mSlideMotor.setTargetPosition(NeptuneConstants.MIN_SAFE_POSTITION_FOR_VBAR);
                switch (mVBarCurrentPosition) {
                    case UP:
                        mVbarServo.setPosition(NeptuneConstants.NEPTUNE_VBAR_TARGET_POSITION_UP);
                        while ( !servoAtPos(NeptuneConstants.NEPTUNE_VBAR_TARGET_POSITION_UP)) {
//                            mOpMode.sleep(10);
                            mSlideMotor.set(MAX_SLIDE_MOTOR_POWER);
                        }
                        break;
                    case DOWN:
                        mVbarServo.setPosition(NeptuneConstants.NEPTUNE_VBAR_TARGET_POSITION_DOWN);
                        while ( !servoAtPos(NeptuneConstants.NEPTUNE_VBAR_TARGET_POSITION_DOWN)) {
//                            mOpMode.sleep(10);
                            mSlideMotor.set(MAX_SLIDE_MOTOR_POWER);
                        }
                        break;
                }
                //wait here for the vbar to change

            }
        }
        mSlideMotor.setTargetPosition(mSlideMotorTargetPosition);
    }
    @Override
    public void periodic(){
        switch (mState){
            case AUTO:
                autoState();
                break;
            case MANUAL:
                break;
        }
    }

    public boolean servoAtPos(double dp){

        double cp = mVbarAnalog.getVoltage();

        cp = -.329*cp+1.04;

        return cp >= (dp * .90) && cp <= (dp * 1.1);
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
    public void changeToNextSlidePosition(){
        if (mSlidesCurrentPosition == SlidesPosition.POSITION_1)
            moveToPosition(SlidesPosition.POSITION_2);
        else if (mSlidesCurrentPosition == SlidesPosition.POSITION_2) {
            moveToPosition(SlidesPosition.POSITION_3);
        }else if (mSlidesCurrentPosition == SlidesPosition.POSITION_3) {
            moveToPosition(SlidesPosition.POSITION_1);
        } else if(mSlidesCurrentPosition == SlidesPosition.HOME_POS){
            moveToPosition(SlidesPosition.POSITION_1);
        }
    }
    public void moveToPosition(SlidesPosition position){
        // anytime the user wants to move to a position we need to be in auto state
        changeSlideState(SlideSubsystemState.AUTO);
        mSlidesCurrentPosition = position;
    }


    public void stopMotorResetEncoder() {
//        mNeptune.mOpMode.telemetry.addLine("Reset Encoder");
//        mNeptune.mOpMode.telemetry.update();
        mSlideMotor.encoder.reset();
        mSlideMotor.set(0);
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
    public boolean isBusy (){
        return !mSlideMotor.atTargetPosition();
    }
    public void addTelemetry(Telemetry telemetry){
        telemetry.addLine(String.format("slide_setting - %s", mSlidesCurrentPosition.toString()));
        telemetry.addLine(String.format("current_position - %d", mSlideMotor.getCurrentPosition()));
        telemetry.addLine(String.format("current_power %.2f", mSlideMotor.motor.getPower()));
        telemetry.addLine(String.format("target_position %d", mSlideMotorTargetPosition));

        telemetry.addLine(String.format("vbar_setting - %s", mVBarCurrentPosition.toString()));
        telemetry.addLine(String.format("vbarposvoltage %.2f", mVbarAnalog.getVoltage()));
        telemetry.addLine(String.format("vbarcurrentpos %d", mVBarMotorTargetPosition));


    }
}
