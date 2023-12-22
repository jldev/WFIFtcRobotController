package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class SlidesSubsystem extends SubsystemBase {

    private double setOuput = 0.0;

    public enum SlidesPosition {
        POSITION_1,
        POSITION_2,
        HOME_POS,
    }

    ;
    private final MotorEx mSlideMotor;
    private final MotorEx mFourBarMotor;


    public SlidesSubsystem(MotorEx slidemotor, MotorEx fourbarmotor) {
        mSlideMotor = slidemotor;
        mFourBarMotor = fourbarmotor;
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mFourBarMotor.setRunMode(MotorEx.RunMode.PositionControl);

        resetPositionCounter();
    }

    public void update() {
        mSlideMotor.set(setOuput);
    }

    private void moveToPosition(int position) {
        mSlideMotor.setTargetPosition(position);
        mSlideMotor.setRunMode(MotorEx.RunMode.PositionControl);
        mSlideMotor.set(position);
        setOuput = position;
    }

    public void moveToPosition(SlidesPosition position){
        if (position == SlidesPosition.POSITION_1){
            moveToPosition(1400);
        }
        if(position == SlidesPosition.HOME_POS){
            moveToPosition(0);
        }
    }

    public boolean isBusy (){
        return !mSlideMotor.atTargetPosition();
    }


    public void resetPositionCounter() {
        mSlideMotor.resetEncoder();
    }
}
