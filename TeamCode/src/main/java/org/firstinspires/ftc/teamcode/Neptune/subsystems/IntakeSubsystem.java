package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum LiftableIntakePosition{
        LOW,
        RAISE,
        P2,
        P3,
        P4,
        P5,
    }

    private final Servo mIntakeLiftServo1;
    private final Servo mIntakeLiftServo2;
    private final Motor mIntakeMotor;
    private final Motor mIntakeMotor2;
    private double mIntakeLiftPosition = 0.0;
    private boolean intakeAutoControl = false;

    public IntakeSubsystem(Neptune neptune, Motor intakeMotor, Motor intakeMotor2, Servo liftServo, Servo liftServo2 ){
        mIntakeMotor = intakeMotor;
        mIntakeMotor2 = intakeMotor2;
        mIntakeMotor.setInverted(true);
        mIntakeMotor2.setInverted(true);
        mIntakeLiftServo1 = liftServo;
        mIntakeLiftServo2 = liftServo2;
        mIntakeLiftServo1.setDirection(Servo.Direction.REVERSE);
        mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS;
        mIntakeLiftServo1.setPosition(mIntakeLiftPosition);
        mIntakeLiftServo2.setPosition(mIntakeLiftPosition);

        if (neptune.mOpModeType == Neptune.OpModeType.AUTO){
            intakeAutoControl = true;
        }


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
                intakeLiftState = LiftableIntakePosition.LOW;
                break;
            case EJECTING:
                mIntakeMotor.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_EJECT_POWER);
                mIntakeMotor2.set(NeptuneConstants.NEPTUNE_INTAKE_MOTOR_EJECT_POWER);
                intakeLiftState = LiftableIntakePosition.RAISE;
                break;
            case NEUTRAL:
                intakeLiftState = LiftableIntakePosition.RAISE;
                mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS;
                mIntakeMotor.stopMotor();
                mIntakeMotor2.stopMotor();
                break;
        }
        if(intakeAutoControl) {
            switch (intakeLiftState) {
                case LOW:
                    mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2;
                    break;
                case P2:
                    mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_P2;
                    break;
                case P3:
                    mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_P3;
                    break;
                case P4:
                    mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_P4;
                    break;
                case P5:
                    mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_P5;
                    break;
                case RAISE:
                    mIntakeLiftPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS;
                    break;
            }
        }
        mIntakeLiftServo1.setPosition(mIntakeLiftPosition);
        mIntakeLiftServo2.setPosition(mIntakeLiftPosition);
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

    public void setIntakeLiftPositionPercentage(double percentage){
        double servoPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS +
                (percentage * (NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2 - NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS));
        mIntakeLiftPosition = servoPosition;
    }
}
