package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum LiftableIntakePosition{
        POSITION_1,
        POSITION_2,
    }

    private final Servo mIntakeLiftServo;
    private final Motor mIntakeMotor;



    public IntakeSubsystem(Motor intakeMotor, Servo liftServo){
        mIntakeMotor = intakeMotor;
        mIntakeLiftServo = liftServo;
    }

    public enum IntakeState {
        INTAKING,
        EJECTING,
        NEUTRAL
    }

    IntakeState intakeState = IntakeState.NEUTRAL;
    LiftableIntakePosition intakeLiftState = LiftableIntakePosition.POSITION_1;
    @Override
    public void periodic(){
        if (intakeFull()){
            intakeState = IntakeState.NEUTRAL;
        }
        switch(intakeState) {
            case INTAKING:
                mIntakeMotor.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_INTAKE_POWER);
                break;
            case EJECTING:
                mIntakeMotor.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_EJECT_POWER);
                break;
            case NEUTRAL:
                mIntakeMotor.stopMotor();
                break;
        }
        switch(intakeLiftState){
            case POSITION_1:
                mIntakeLiftServo.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS1);
                break;
            case POSITION_2:
                mIntakeLiftServo.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2);
                break;
        }
    }

    public void intakeOn(){
        intakeState = IntakeState.INTAKING;
    }

    public void intakeOff(){
        intakeState = IntakeState.NEUTRAL;
    }

    public void intakeEject(){
        intakeState = IntakeState.EJECTING;
    }
    public boolean intakeFull(){
        return false;
    }

    public void intakeLift(LiftableIntakePosition position){
        intakeLiftState = position;
    }
}
