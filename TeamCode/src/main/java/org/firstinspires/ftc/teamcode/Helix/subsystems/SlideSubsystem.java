package org.firstinspires.ftc.teamcode.Helix.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;

public class SlideSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Helix mHelix;
    private int mVerticleTargetPosiion = 0;
    private int mHorizontalTargetPosiion = 0;

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
        BASKET,
        PRELOAD_BASKET
    }



    SlideSubsystemState mState;

    public SlidePosition verticlePosition;
    public SlidePosition horizontalPosition;

    private final MotorEx mVerticalSlideMotor;
    private final PIDFController mVerticalPIDController;
    private final MotorEx mHorizontalSlideMotor;
    private final PIDFController mHorizontalPIDController;

    public SlideSubsystem(Helix helix, MotorEx verticalSlideMotor, MotorEx horizontalSlideMotor, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
        mHelix = helix;
        mVerticalSlideMotor = verticalSlideMotor;
        mOpMode = opmode;
        mVerticalPIDController = new PIDFController(HelixConstants.VERTICAL_PID_P, HelixConstants.VERTICAL_PID_I,
                HelixConstants.VERTICAL_PID_D, HelixConstants.VERTICAL_PID_F);
        mVerticalPIDController.setTolerance(HelixConstants.SLIDES_PID_TOLERANCE);
        mHorizontalPIDController = new PIDFController(HelixConstants.HORIZONTAL_PID_P, HelixConstants.HORIZONTAL_PID_I,
                HelixConstants.HORIZONTAL_PID_D, HelixConstants.HORIZONTAL_PID_F);
        mHorizontalPIDController.setTolerance(HelixConstants.SLIDES_PID_TOLERANCE)
        ;
        mVerticalSlideMotor.stopAndResetEncoder();
        mVerticalSlideMotor.setRunMode(MotorEx.RunMode.RawPower);
        mVerticalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mVerticalPIDController.setSetPoint(0);
        verticlePosition = SlidePosition.HOME;
        mVerticleTargetPosiion = 0;
        mVerticalSlideMotor.resetEncoder();
        mVerticalSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mVerticalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
        mState = SlideSubsystemState.AUTO;

        mHorizontalSlideMotor = horizontalSlideMotor;
        mHorizontalSlideMotor.stopAndResetEncoder();
        mHorizontalSlideMotor.setRunMode(MotorEx.RunMode.RawPower);
        mHorizontalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mHorizontalPIDController.setSetPoint(0);
        horizontalPosition = SlidePosition.HOME;
        mHorizontalTargetPosiion = 0;
        mHorizontalSlideMotor.resetEncoder();
        mHorizontalSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mHorizontalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
        opmode.telemetry.addLine("Slide Init");
        opmode.telemetry.update();
    }


    @Override
    public void periodic() {
        if (mState == SlideSubsystemState.AUTO) {

                switch (verticlePosition) {
                    case HOME:
                        mVerticleTargetPosiion = HelixConstants.VERTICAL_SLIDE_HOME;
                        mHorizontalTargetPosiion = HelixConstants.HORIZONTAL_SLIDE_HOME;
                        break;
                    case WALL:
                        mVerticleTargetPosiion = HelixConstants.VERTICAL_SLIDE_WALL;
                        mHorizontalTargetPosiion = HelixConstants.HORIZONTAL_SLIDE_WALL;
                        break;
                    case HANG:
                        mVerticleTargetPosiion = HelixConstants.VERTICAL_SLIDE_HANG;
                        mHorizontalTargetPosiion = HelixConstants.HORIZONTAL_SLIDE_HANG;
                        break;
                    case BASKET:
                        mVerticleTargetPosiion = HelixConstants.VERTICAL_SLIDE_BASKET;
                        mHorizontalTargetPosiion = HelixConstants.HORIZONTAL_SLIDE_BASKET;
                        break;
                    case PRELOAD_BASKET:
                        mVerticleTargetPosiion = HelixConstants.VERTICAL_PRELOAD_BASKET;
                        mHorizontalTargetPosiion = HelixConstants.HORIZONTAL_PRELOAD_BASKET;
                        break;
            }
        } else {
            switch (mVerticalManualDirection) {
                case UP:
                    mVerticleTargetPosiion += HelixConstants.SLIDE_MANUAL_SPEED;
                    mVerticalPIDController.setP(HelixConstants.VERTICAL_PID_P);
                    break;
                case DOWN:
                    mVerticleTargetPosiion -= HelixConstants.SLIDE_MANUAL_SPEED;
                    mVerticalPIDController.setP(0.015);
                    break;
                case OFF:
                    break;
            }

            switch (mHorizontalManualDirection) {
                case OUT:
                    mHorizontalTargetPosiion += HelixConstants.SLIDE_MANUAL_SPEED;
                    break;
                case IN:
                    mHorizontalTargetPosiion -= HelixConstants.SLIDE_MANUAL_SPEED;
                    break;
                case OFF:
                    break;
            }

            if(mHorizontalTargetPosiion < 0.00)
                mHorizontalTargetPosiion = 0;
            if(mVerticleTargetPosiion < 0.00)
                mVerticleTargetPosiion = 0;
        }

        mVerticalPIDController.setSetPoint(mVerticleTargetPosiion);
        double output = mVerticalPIDController.calculate(
                mVerticalSlideMotor.getCurrentPosition());
            mVerticalSlideMotor.set(output);


        mHorizontalPIDController.setSetPoint(mHorizontalTargetPosiion);
        output = mHorizontalPIDController.calculate(
                mHorizontalSlideMotor.getCurrentPosition());
        if (!mHorizontalPIDController.atSetPoint()) {
            mHorizontalSlideMotor.set(output);
        } else{
            mHorizontalSlideMotor.stopMotor(); // stop the motor
        }



    }

    private void changeSlideState(SlideSubsystemState newState){
        mState = newState;
    }


    public void changeToSlidePosition(SlidePosition pos){
        verticlePosition = pos;
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
        mVerticalPIDController.setSetPoint(0);
        mVerticalPIDController.reset();
        mVerticalSlideMotor.stopMotor();
        mVerticalSlideMotor.resetEncoder();

//        mHorizontalPIDController.clearTotalError();
//        mHorizontalPIDController.setSetPoint(0);
//        mHorizontalSlideMotor.stopMotor();
//        mHorizontalSlideMotor.resetEncoder();
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
        return !mVerticalPIDController.atSetPoint() || !mHorizontalPIDController.atSetPoint();
    }

    public void addTelemetry(Telemetry telemetry){
        mOpMode.telemetry.addData("vCurrent: ", mVerticalSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("vTarget: ", mVerticleTargetPosiion);
        mOpMode.telemetry.addData("hCurrent: ", mHorizontalSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("hTarget: ", mHorizontalTargetPosiion);


    }
}