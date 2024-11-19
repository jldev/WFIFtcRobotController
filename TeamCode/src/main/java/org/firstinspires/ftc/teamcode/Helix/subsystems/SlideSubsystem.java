package org.firstinspires.ftc.teamcode.Helix.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        BASKET
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
        mVerticalSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        mVerticalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mVerticalPIDController.setSetPoint(0);
        verticlePosition = SlidePosition.HOME;
        mVerticleTargetPosiion = 0;
        mVerticalSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mVerticalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
        mState = SlideSubsystemState.AUTO;

        mHorizontalSlideMotor = horizontalSlideMotor;
        mHorizontalSlideMotor.stopAndResetEncoder();
        mHorizontalSlideMotor.setRunMode(MotorEx.RunMode.VelocityControl);
        mHorizontalSlideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mHorizontalPIDController.setSetPoint(0);
        horizontalPosition = SlidePosition.HOME; // !! linked with vertical !!
        mHorizontalTargetPosiion = 0; // !! linked with vertical !!
        mHorizontalSlideMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mHorizontalSlideMotor.encoder.setDirection(Motor.Direction.FORWARD);
    }


    @Override
    public void periodic() {
        if (mState == SlideSubsystemState.AUTO) {

                switch (verticlePosition) {
                    case HOME:
                        mVerticleTargetPosiion = HelixConstants.SLIDE_HOME;
                        break;
                    case WALL:
                        mVerticleTargetPosiion = HelixConstants.SLIDE_WALL;
                        break;
                    case HANG:
                        mVerticleTargetPosiion = HelixConstants.SLIDE_HANG;
                        break;
                    case BASKET:
                        mVerticleTargetPosiion = HelixConstants.SLIDE_BASKET;
                        break;
            }
        } else {
            switch (mVerticalManualDirection) {
                case UP:
                    mVerticleTargetPosiion += HelixConstants.SLIDE_MANUAL_SPEED;
                    break;
                case DOWN:
                    mVerticleTargetPosiion -= HelixConstants.SLIDE_MANUAL_SPEED;
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
        }
        /*
            If we are running fast enough the below while loops could be removed so we
            don't hold up the rest of the system, but for testing and tuning the PID controllers
            this should work for now
         */
        mVerticalPIDController.setSetPoint(mVerticleTargetPosiion);
        while (!mVerticalPIDController.atSetPoint()) {
            double output = mVerticalPIDController.calculate(
                    mVerticalSlideMotor.getCurrentPosition()  // the measured value
            );
            mVerticalSlideMotor.setVelocity(output);
        }
        mVerticalSlideMotor.stopMotor(); // stop the motor

        mHorizontalPIDController.setSetPoint(mHorizontalTargetPosiion);
        while (!mHorizontalPIDController.atSetPoint()) {
            double output = mHorizontalPIDController.calculate(
                    mHorizontalSlideMotor.getCurrentPosition()  // the measured value
            );
            mHorizontalSlideMotor.setVelocity(output);
        }
        mHorizontalSlideMotor.stopMotor(); // stop the motor

        mOpMode.telemetry.addData("vCurrent: ", mVerticalSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("vTarget: ", mVerticleTargetPosiion);
        mOpMode.telemetry.addData("hCurrent: ", mHorizontalSlideMotor.encoder.getPosition());
        mOpMode.telemetry.addData("hTarget: ", mHorizontalTargetPosiion);
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
        mVerticalPIDController.clearTotalError();
        mVerticalPIDController.setSetPoint(0);
        mVerticalSlideMotor.stopMotor();
        mVerticalSlideMotor.resetEncoder();
        mHorizontalPIDController.clearTotalError();
        mHorizontalPIDController.setSetPoint(0);
        mHorizontalSlideMotor.stopMotor();
        mHorizontalSlideMotor.resetEncoder();
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