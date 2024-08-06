package org.firstinspires.ftc.teamcode.Helix.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;

public class IntakeSubsystem extends SubsystemBase {



    private final Servo mLiftServo;
    private final Servo mGripperServo;
    private double mIntakeLiftPosition = 0.0;
    private boolean intakeAutoControl = false;

    public IntakeSubsystem(Helix helix, Servo liftServo, Servo gripperServo){
        mLiftServo = liftServo;
        mGripperServo = gripperServo;
        mLiftServo.setDirection(Servo.Direction.REVERSE);
        mIntakeLiftPosition = HelixConstants.LIFT_POS_0;
        mLiftServo.setPosition(mIntakeLiftPosition);
        mGripperServo.setPosition(mIntakeLiftPosition);

        if (helix.mOpModeType == Helix.OpModeType.AUTO){
            intakeAutoControl = true;
        }


    }

    public InstantCommand setLift() {
        if(liftState == LiftState.P0)
            return new InstantCommand(() -> liftState = LiftState.P0 );

        else
            return new InstantCommand(() -> liftState = LiftState.P1 );

    }


    public enum GripperState {
        CLOSED,

        OPEN,
    }

    public enum LiftState {
        P0,
        P1,
    }




    GripperState gripperState = GripperState.CLOSED;
    LiftState liftState = LiftState.P0;
    @Override
    public void periodic(){


        switch(gripperState) {
            case OPEN:
                mGripperServo.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
                break;
            case CLOSED:
                mGripperServo.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
                break;
        }
            switch (liftState) {
                case P0:
                    mIntakeLiftPosition = HelixConstants.LIFT_POS_0;
                    break;
                case P1:
                    mIntakeLiftPosition = HelixConstants.LIFT_POS_1;
                    break;
            }
        mLiftServo.setPosition(mIntakeLiftPosition);
    }

    public boolean intakeFull(){
        return false;
    }




    public InstantCommand setGripperOpen(){
        return new InstantCommand(() -> gripperState = GripperState.OPEN );
    }
    public InstantCommand setGripperClosed(){
        return new InstantCommand(() -> gripperState = GripperState.CLOSED );
    }

    public InstantCommand setLiftP0(){
        return new InstantCommand(() -> liftState = LiftState.P0);
    }
    public InstantCommand setLiftP1(){
        return new InstantCommand(() -> liftState = LiftState.P1);
    }






    public void setGripperState(GripperState state){
        gripperState = state;
    }

    public void setLiftState(LiftState position){
        liftState = position;
    }

//    public void setIntakeLiftPositionPercentage(double percentage){
//        double servoPosition = NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS +
//                (percentage * (NeptuneConstants.NEPTUNE_INTAKE_SERVO_POS2 - NeptuneConstants.NEPTUNE_INTAKE_SERVO_INITIAL_POS));
//        mIntakeLiftPosition = servoPosition;
//    }
}
