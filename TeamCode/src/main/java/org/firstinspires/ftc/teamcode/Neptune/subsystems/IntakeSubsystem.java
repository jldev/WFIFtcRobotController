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
    private final Motor mIntakeMotor2;



    public IntakeSubsystem(Motor intakeMotor, Motor intakeMotor2, Servo liftServo, Servo liftServo2 ){
        mIntakeMotor = intakeMotor;
        mIntakeMotor2 = intakeMotor2;
        mIntakeMotor.setInverted(true);
        mIntakeMotor2.setInverted(true);
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
    LiftableIntakePosition intakeLiftState = LiftableIntakePosition.RAISE;
    @Override
    public void periodic(){
        if (intakeFull()){
            intakeState = IntakeState.NEUTRAL;
        }
        switch(intakeState) {
            case INTAKING:
                mIntakeMotor.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_INTAKE_POWER);
                mIntakeMotor2.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_INTAKE_POWER);
                intakeLiftState = LiftableIntakePosition.LOWER;
                break;
            case EJECTING:
                mIntakeMotor.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_EJECT_POWER);
                mIntakeMotor2.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_EJECT_POWER);
                intakeLiftState = LiftableIntakePosition.RAISE;
                break;
            case NEUTRAL:
                intakeLiftState = LiftableIntakePosition.RAISE;
                mIntakeMotor.stopMotor();
                mIntakeMotor2.stopMotor();
                break;
        }
        switch(intakeLiftState){
            case LOWER:
                mIntakeLiftServo1.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2);
                mIntakeLiftServo2.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2);
                break;
            case RAISE:
                mIntakeLiftServo1.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS);
                mIntakeLiftServo2.setPosition(NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS);
                break;
        }
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
