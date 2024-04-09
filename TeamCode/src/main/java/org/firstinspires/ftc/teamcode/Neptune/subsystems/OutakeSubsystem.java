package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class OutakeSubsystem extends SubsystemBase {

//TODO: ADD NEW CODE FOR VBAR SERVO

    private final Servo mOutakeServo;


    public OutakeSubsystem(Servo outakeServo){
        mOutakeServo = outakeServo;


        mOutakeServo.setPosition(NeptuneConstants.OUTAKE_CLOSED_POSITION);
    }

    public enum OutakeState {
        OPENED,
        CLOSED,
    }

    public enum AutoOutakeState {
        OPENED,
        CLOSED,
    }

      OutakeState outakeState = OutakeState.CLOSED;
     AutoOutakeState autoOutakeState = AutoOutakeState.CLOSED;

    @Override
    public void periodic(){
        switch(outakeState) {
            case OPENED:
                mOutakeServo.setPosition(NeptuneConstants.OUTAKE_OPEN_POSITION);
                break;
            case CLOSED:
                mOutakeServo.setPosition(NeptuneConstants.OUTAKE_CLOSED_POSITION);
                break;
        }
    }

    public void setOutakeState(OutakeState state){
        outakeState = state;
    }
    public void setAutoOutakeState(AutoOutakeState state){
        autoOutakeState = state;
    }
}
