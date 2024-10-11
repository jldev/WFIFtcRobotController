package org.firstinspires.ftc.teamcode.Helix.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;

import java.util.function.BooleanSupplier;

public class SlideSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Helix mHelix;
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
    public enum SlidePosition{
        HOME,
        TEMP_UP,
    }

    public enum VBarPosition {
        UP,
        DOWN
    }

    SlideSubsystemState mState;

    private SlidePosition slidePosition;

    private final MotorEx mSlideMotor;

    public SlideSubsystem(Helix helix, MotorEx slideMotor, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
        mHelix = helix;
        mSlideMotor = slideMotor;
        mOpMode = opmode;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mSlideMotor.setPositionCoefficient(pos_coefficient);
        mSlideMotor.setPositionTolerance(pos_tolerance);
        mSlideMotor.setTargetPosition(mSlideMotorTargetPosition);
        mSlideMotor.resetEncoder();
        slidePosition = SlidePosition.HOME;
        mSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mSlideMotor.encoder.setDirection(Motor.Direction.REVERSE);
        mState = SlideSubsystemState.AUTO;
    }


    @Override
    public void periodic(){
        if (mState == SlideSubsystemState.AUTO){

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


    public void changeToSlidePosition(){
        switch(slidePosition){
            case HOME:
                //go down
                break;
            case TEMP_UP:
                //go up
                break;
        }
    }





//    public void moveToPosition(SlidesPosition position){
//        // anytime the user wants to move to a position we need to be in auto state
//        changeSlideState(SlideSubsystemState.AUTO);
//        mSlidesCurrentPosition = position;
//    }


    public void stopMotorResetEncoder() {
//        mNeptune.mOpMode.telemetry.addLine("Reset Encoder");
//        mNeptune.mOpMode.telemetry.update();
        mSlideMotor.set(0);
        mSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        mSlideMotor.resetEncoder();
    }
    public void manualSlideControl(ManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        switch (direction){
            case UP:
                mSlideMotor.set(HelixConstants.SLIDE_MOTOR_MANUAL_POWER);
                break;
            case DOWN:
                mSlideMotor.set(-HelixConstants.SLIDE_MOTOR_MANUAL_POWER);
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
        telemetry.addLine(String.format("slide_setting - %s", slidePosition.toString()));
        telemetry.addLine(String.format("current_position - %d", mSlideMotor.getCurrentPosition()));
        telemetry.addLine(String.format("current_power %.2f", mSlideMotor.motor.getPower()));
        telemetry.addLine(String.format("target_position %d", mSlideMotorTargetPosition));


    }
}