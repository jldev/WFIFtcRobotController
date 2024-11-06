package org.firstinspires.ftc.teamcode.Helix.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;

public class SlideSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Helix mHelix;
    private int mSlideMotorTargetPosition = 0;
    private int mVBarMotorTargetPosition;

    private final CommandOpMode mOpMode;
    private VerticalManualControlDirection mVerticalManualDirection = VerticalManualControlDirection.OFF;
    private HorizontalManualControlDirection mHorizontalManualDirection = HorizontalManualControlDirection.OFF;

    public enum SlideSubsystemState {
        AUTO,
        MANUAL
    }

    public enum VerticalManualControlDirection {
        UP,
        DOWN,
        OFF
    }

    public enum HorizontalManualControlDirection {
        OUT,
        IN,
        OFF
    }


    public enum SlidePosition{
        HOME,
        WALL,
        HANG,
        BASKET
    }



    SlideSubsystemState mState;

    public SlidePosition slidePosition;

    private final MotorEx mVerticalSlideMotor;
    private final MotorEx mHorizontalSlideMotor;

    public SlideSubsystem(Helix helix, MotorEx verticalSlideMotor, MotorEx horizontalSlideMotor, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
        mHelix = helix;
        mVerticalSlideMotor = verticalSlideMotor;
        mOpMode = opmode;
        mVerticalSlideMotor.stopAndResetEncoder();
        mVerticalSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mVerticalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mVerticalSlideMotor.setPositionCoefficient(pos_coefficient);
        mVerticalSlideMotor.setPositionTolerance(pos_tolerance);
        mVerticalSlideMotor.setTargetPosition(0);
        slidePosition = SlidePosition.HOME;
        mSlideMotorTargetPosition = 0;
        mVerticalSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mVerticalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
        mState = SlideSubsystemState.AUTO;

        mHorizontalSlideMotor = horizontalSlideMotor;
        mHorizontalSlideMotor.stopAndResetEncoder();
        mHorizontalSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mHorizontalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mHorizontalSlideMotor.setPositionCoefficient(pos_coefficient);
        mHorizontalSlideMotor.setPositionTolerance(pos_tolerance);
        mHorizontalSlideMotor.setTargetPosition(0);
        slidePosition = SlidePosition.HOME; // !! linked with vertical !!
        mSlideMotorTargetPosition = 0; // !! linked with vertical !!
        mHorizontalSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mHorizontalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
    }


    @Override
    public void periodic() {
        if (mState == SlideSubsystemState.AUTO) {
            if (!mVerticalSlideMotor.atTargetPosition()) {
                mVerticalSlideMotor.set(HelixConstants.SLIDE_SPEED);
            } else {
                mVerticalSlideMotor.set(0.001);
                switch (slidePosition) {
                    case HOME:
                        mSlideMotorTargetPosition = HelixConstants.SLIDE_HOME;
                        break;
                    case WALL:
                        mSlideMotorTargetPosition = HelixConstants.SLIDE_WALL;
                        break;
                    case HANG:
                        mSlideMotorTargetPosition = HelixConstants.SLIDE_HANG;
                        break;
                    case BASKET:
                        mSlideMotorTargetPosition = HelixConstants.SLIDE_BASKET;
                        break;
                }
                mVerticalSlideMotor.setTargetPosition(mSlideMotorTargetPosition);
            }
        } else {
            switch (mVerticalManualDirection) {
                case UP:
                    mVerticalSlideMotor.set(HelixConstants.SLIDE_SPEED);
                    break;
                case DOWN:
                    mVerticalSlideMotor.set(-HelixConstants.SLIDE_SPEED);
                    break;
                case OFF:
                    mVerticalSlideMotor.set(0);
                    break;
            }

            switch (mHorizontalManualDirection) {
                case OUT:
                    mHorizontalSlideMotor.set(-HelixConstants.SLIDE_SPEED);
                    break;
                case IN:
                    mHorizontalSlideMotor.set(HelixConstants.SLIDE_SPEED);
                    break;
                case OFF:
                    mHorizontalSlideMotor.set(0);
                    break;
            }
        }
    }

    private void changeSlideState(SlideSubsystemState newState){
        if (mState != newState){ //we need to change the state
            if (newState == SlideSubsystemState.AUTO){
                mVerticalSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
                mHorizontalSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
            } else {
                //we are changing to MANUAL
                mVerticalSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
                mHorizontalSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
            }
        }
        mState = newState;
    }


    public void changeToSlidePosition(SlidePosition pos){
        slidePosition = pos;
        changeSlideState(SlideSubsystemState.AUTO);
    }





//    public void moveToPosition(SlidesPosition position){
//        // anytime the user wants to move to a position we need to be in auto state
//        changeSlideState(SlideSubsystemState.AUTO);
//        mSlidesCurrentPosition = position;
//    }


    public void stopMotorResetEncoder() {
//        mNeptune.mOpMode.telemetry.addLine("Reset Encoder");
//        mNeptune.mOpMode.telemetry.update();
        mVerticalSlideMotor.set(0);
        mVerticalSlideMotor.setRunMode(Motor.RunMode.PositionControl);
        mVerticalSlideMotor.resetEncoder();
    }




    public void verticalManualSlideControl(VerticalManualControlDirection direction){
        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        mVerticalManualDirection = direction;
    }

    public void horizontalManualSlideControl(HorizontalManualControlDirection direction){

        // anytime the user want to manual control we need to be in the manual state
        changeSlideState(SlideSubsystemState.MANUAL);
        mHorizontalManualDirection = direction;
    }


    public boolean isBusy (){
        return !mVerticalSlideMotor.atTargetPosition();
    }
//    public void addTelemetry(Telemetry telemetry){
//        telemetry.addLine(String.format("Slide State - %s", mState));
//        telemetry.addLine(String.format("slide_setting - %s", slidePosition.toString()));
//        telemetry.addLine(String.format("current_position - %d", mVerticalSlideMotor.getCurrentPosition()));
//        telemetry.addLine(String.format("current_power %.2f", mVerticalSlideMotor.motor.getPower()));
//        telemetry.addLine(String.format("target_position %d", mSlideMotorTargetPosition));
//
//
//    }
}