package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum LiftableIntakePosition{
        RAISE,
        LOWER,
    }

    private final Servo mIntakeLiftServo1;
    private final Servo mIntakeLiftServo2;
    private final Motor mIntakeMotor;



    public IntakeSubsystem(Motor intakeMotor, Servo liftServo, Servo liftServo2 ){
        mIntakeMotor = intakeMotor;
        mIntakeLiftServo1 = liftServo;
        mIntakeLiftServo2 = liftServo2;
        mIntakeLiftServo1.setDirection(Servo.Direction.REVERSE);
        mIntakeLiftServo1.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS);
        mIntakeLiftServo2.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS);

    }

    public enum IntakeState {
        INTAKING,
        EJECTING,
        NEUTRAL
    }

    IntakeState intakeState = IntakeState.NEUTRAL;
    LiftableIntakePosition intakeLiftState = LiftableIntakePosition.LOWER;
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
            case LOWER:
                mIntakeLiftServo1.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2);
                mIntakeLiftServo2.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2);
                break;
            case RAISE:
                mIntakeLiftServo1.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS1);
                mIntakeLiftServo2.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS1);
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

    public void setIntakeState(IntakeSubsystem.IntakeState state){
        intakeState = state;
    }

    public void setIntakeLiftState(LiftableIntakePosition position){
        intakeLiftState = position;
    }
}
