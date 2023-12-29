package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class OutakeSubsystem extends SubsystemBase {



    private final Servo mOutakeServo;



    public OutakeSubsystem(Servo outakeServo){
        mOutakeServo = outakeServo;
    }

    public enum OutakeState {
        OPENED,
        CLOSED,
//        NEUTRAL
    }

//    OutakeState outakeState = OutakeState.NEUTRAL;
      OutakeState outakeState = OutakeState.CLOSED;

    @Override
    public void periodic(){
//        if (intakeFull()){
//            outakeState = outakeState   .NEUTRAL;
//        }
        switch(outakeState) {
            case OPENED:
//                mOutakeServo.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_INTAKE_POWER);
                mOutakeServo.setPosition(NeptuneConstants.OUTAKE_OPEN_POSITION);
                break;
            case CLOSED:
//                mOutakeServo.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_EJECT_POWER);
                mOutakeServo.setPosition(NeptuneConstants.OUTAKE_CLOSED_POSITION);
                break;
//            case NEUTRAL:
//                mIntakeMotor.stopMotor();
//                break;
        }
        }

    public void setOutakeState(OutakeState state){
        outakeState = state;
    }
}
