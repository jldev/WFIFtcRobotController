package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class HangSubsystem extends SubsystemBase {

    public enum HangPos {
        POSITION_1,
    }


    private final Motor mHangMotor;


    public HangSubsystem(Motor hangmotor) {
        mHangMotor = hangmotor;
    }

    public enum HangState {
        HANGING,
        REST,
    }

    HangState hangstate = HangState.REST;
    HangPos hangPosition = HangPos.POSITION_1;

    @Override
    public void periodic() {
        if ((hangatrest())) {
            hangstate = hangstate.REST;
        }
        switch (hangstate) {
            case HANGING:
                mHangMotor.set(NeptuneConstants.NEPTUNE_HANG_MOTOR_POWER);
                break;
        }
    }

    public void hangOff(){
        hangstate = HangState.REST;
    }

    public boolean hangatrest(){
        return false;
    }

}
