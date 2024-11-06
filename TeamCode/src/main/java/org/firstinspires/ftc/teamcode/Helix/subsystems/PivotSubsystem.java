package org.firstinspires.ftc.teamcode.Helix.subsystems;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;

public class PivotSubsystem extends SubsystemBase {
    //private final Trigger encoderStopTrigger;
    private final Helix mHelix;

    private final CommandOpMode mOpMode;


    public enum DesiredColor {
        RED,
        BLUE,
        GREEN,
        NULL,
    }


    public DesiredColor desiredColor;
    public DesiredColor lastDesiredColor;

    private final MotorEx mPivotMotor;

    public PivotSubsystem(Helix helix, MotorEx pivotMotor, CommandOpMode opmode, double pos_coefficient, double pos_tolerance) {
        mHelix = helix;
        mPivotMotor = pivotMotor;
        mOpMode = opmode;
        mPivotMotor.stopAndResetEncoder();
        mPivotMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mPivotMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mPivotMotor.setPositionCoefficient(pos_coefficient);
        mPivotMotor.setPositionTolerance(pos_tolerance);
        mPivotMotor.setTargetPosition(0);
        desiredColor = DesiredColor.NULL;
        lastDesiredColor = DesiredColor.NULL;
        mPivotMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        mPivotMotor.encoder.setDirection(Motor.Direction.FORWARD);
    }


    @Override
    public void periodic() {
        mPivotMotor.set(0.001);
        switch (desiredColor) {
            case RED:
                // +pivot until we get to red
                mPivotMotor.set(HelixConstants.PIVOT_SPEED);
                break;
            case BLUE:
                // -pivot until we get to blue
                mPivotMotor.set(-HelixConstants.PIVOT_SPEED);
                break;
            case GREEN:
                // if we were last at red, -pivot & if we were last at blue, +pivot
                if (lastDesiredColor == DesiredColor.RED) {
                    mPivotMotor.set(-HelixConstants.PIVOT_SPEED);
                } else if (lastDesiredColor == DesiredColor.BLUE) {
                    mPivotMotor.set(HelixConstants.PIVOT_SPEED);
                }
                break;
            case NULL:
                mPivotMotor.set(0);
                break;
        }
    }


    public void changeDesiredColor(DesiredColor color) {
        lastDesiredColor = desiredColor;
        desiredColor = color;
    }


//    public void moveToPosition(SlidesPosition position){
//        // anytime the user wants to move to a position we need to be in auto state
//        changeSlideState(SlideSubsystemState.AUTO);
//        mSlidesCurrentPosition = position;
//    }


    public void stopMotorResetEncoder() {
//        mNeptune.mOpMode.telemetry.addLine("Reset Encoder");
//        mNeptune.mOpMode.telemetry.update();
        mPivotMotor.set(0);
        mPivotMotor.setRunMode(Motor.RunMode.PositionControl);
        mPivotMotor.resetEncoder();
    }


    public boolean isBusy() {
        return !mPivotMotor.atTargetPosition();
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