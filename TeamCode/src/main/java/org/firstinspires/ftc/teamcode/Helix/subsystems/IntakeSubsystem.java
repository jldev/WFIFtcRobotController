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

    private Helix mHelix;

    public IntakeSubsystem(Helix helix, Servo liftServo, Servo gripperServo) {
        mLiftServo = liftServo;
        mGripperServo = gripperServo;
        mLiftServo.setDirection(Servo.Direction.REVERSE);
        mIntakeLiftPosition = HelixConstants.LIFT_POS_0;
        mLiftServo.setPosition(mIntakeLiftPosition);
        mGripperServo.setPosition(mIntakeLiftPosition);
        mHelix = helix;

        if (helix.mOpModeType == Helix.OpModeType.AUTO) {
            intakeAutoControl = true;
        }


    }


    public enum GripperState {
        CLOSED,

        OPEN,
    }

    public enum LiftState {
        P0,
        P1,
        P2,
    }


    GripperState gripperState = GripperState.CLOSED;
    LiftState liftState = LiftState.P0;

    @Override
    public void periodic() {


        switch (gripperState) {
            case OPEN:
                mGripperServo.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
                break;
            case CLOSED:
                mGripperServo.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
                break;
        }

    }

    public boolean intakeFull() {
        return false;
    }


    public InstantCommand setGripperOpen() {
        return new InstantCommand(() -> gripperState = GripperState.OPEN);
    }

    public InstantCommand setGripperClosed() {
        return new InstantCommand(() -> gripperState = GripperState.CLOSED);
    }

    public void cycleLift() {
        mHelix.mOpMode.telemetry.addLine("put a string in there " + mHelix.mOpMode.getRuntime());
        mHelix.mOpMode.telemetry.update();

        if (liftState == LiftState.P0) {
            liftState = LiftState.P1;
            mLiftServo.setPosition(HelixConstants.LIFT_POS_1);
        }
        else if (liftState == LiftState.P1) {
            liftState = LiftState.P2;
            mLiftServo.setPosition(HelixConstants.LIFT_POS_2);
        }
        else if (liftState == LiftState.P2) {
            liftState = LiftState.P0;
            mLiftServo.setPosition(HelixConstants.LIFT_POS_0);
        }
    }

    // temp logic
    public void setLift(double position)
    {
        mLiftServo.setPosition(HelixConstants.LIFT_POS_1);
    }
}
