package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class HangSubsystem extends SubsystemBase {

    public enum HangPos {
        POSITION_1,
    }


    private final Servo mHangServo;
    private final Servo mHangServo2;
    private final MotorEx mHangMotor;


    public HangSubsystem(Servo hangservo, Servo hangservo2, MotorEx hangmotor) {
        mHangServo = hangservo;
        mHangServo2 = hangservo2;
        mHangMotor = hangmotor;

        mHangServo2.setDirection(Servo.Direction.REVERSE);
        mHangServo.setPosition(NeptuneConstants.NEPTUNE_HANG_REST_POS);
        mHangServo2.setPosition(NeptuneConstants.NEPTUNE_HANG_REST_POS);
    }

    public enum HangState {
        HANGING,
        REST,
    }

    public enum HangMotorDirection {
        UP,
        DOWN,
        STOPPED,
    }

    HangState hangstate = HangState.REST;

    @Override
    public void periodic() {
        if ((hangatrest())) {
            hangstate = hangstate.REST;
        }
        switch (hangstate) {
            case HANGING:
                mHangServo.setPosition(NeptuneConstants.NEPTUNE_HANG_POS);
                mHangServo2.setPosition(NeptuneConstants.NEPTUNE_HANG_POS);

                break;

            case REST:
                mHangServo.close();
                mHangServo2.close();
//                mHangServo.setPosition(NeptuneConstants.NEPTUNE_HANG_REST_POS);
//                mHangServo2.setPosition(NeptuneConstants.NEPTUNE_HANG_REST_POS);


                break;
        }
    }

    public void hangOff(){
        hangstate = HangState.REST;
    }

    public void setHangState(HangSubsystem.HangState state){
        hangstate = state;
    }

    public void hangDirection(HangMotorDirection hangingDirection){
        switch (hangingDirection) {
            case UP:
                mHangMotor.set(NeptuneConstants.NEPTUNE_HANG_MOTOR_UP_POWER);
                break;
            case DOWN:
                mHangMotor.set(NeptuneConstants.NEPTUNE_HANG_MOTOR_DOWN_POWER);
                break;
            case STOPPED:
                mHangMotor.stopMotor();
                break;
        }
    }



    public boolean hangatrest(){
        return false;
    }

}
