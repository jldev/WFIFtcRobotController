package org.firstinspires.ftc.teamcode.Helix.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;
import org.firstinspires.ftc.teamcode.Helix.drive.SampleMecanumDrive;

import java.util.List;

public class ClawSubsystem extends SubsystemBase {

    private final Helix mHelix;

    private final CommandOpMode mOpMode;

    private final Servo yaw;
    private final Servo pitch;
    private final Servo grip;

    public enum ClawState {
        HOME,
        HANG,
        BASKET,
        SUB
    }

    public enum GripState {
        OPEN,
        CLOSED
    }
    public GripState mGripState;

    private double desiredYaw;
    private double desiredPitch;

    public ClawSubsystem(Helix helix, CommandOpMode commandOpMode, Servo yaw_1, Servo pitch_2, Servo grip_3, Limelight3A limelight_) {
        mHelix = helix;
        mOpMode = commandOpMode;

        yaw = yaw_1;
        pitch = pitch_2;
        grip = grip_3;

        if(helix.mOpModeType == Helix.OpModeType.AUTO)
        {
            yaw.setPosition(HelixConstants.CLAW_YAW_INIT);
            pitch.setPosition(HelixConstants.CLAW_PITCH_INIT);
            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
        }
        yaw.setPosition(yaw.getPosition());
        pitch.setPosition(pitch.getPosition());
        grip.setPosition(grip.getPosition());
    }

    @Override
    public void periodic() {

//        desiredYaw = (mHelix.gunnerOp.getLeftX() / 2) + .5;
//        desiredPitch = (mHelix.gunnerOp.getLeftY() / 2) + .5;





        //   GUNNER CONTROL


        desiredYaw = yaw.getPosition();
        desiredPitch = pitch.getPosition();

        if (Math.abs(mHelix.gunnerOp.getLeftX()) > 0.1 || Math.abs(mHelix.gunnerOp.getLeftY()) > 0.1) {
            desiredYaw = yaw.getPosition() + mHelix.gunnerOp.getLeftX() * .045;
            desiredPitch = pitch.getPosition() + mHelix.gunnerOp.getLeftY() * .045;
        }


        if (desiredYaw > 1.00)
            desiredYaw = 1.00;
        if (desiredYaw < 0.00)
            desiredYaw = 0.00;

        if (desiredPitch > .58)
            desiredPitch = .58;
        if (desiredPitch < 0.00)
            desiredPitch = 0.00;


//        if(mHelix.gunnerOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
//        {
//            switch (desiredColor) {
//                case YELLOW: desiredColor = SampleColor.RED; break;
//                case RED: desiredColor = SampleColor.BLUE; break;
//                case BLUE: desiredColor = SampleColor.YELLOW; break;
//            }
//        }


        // After we get our positions from manual or auto - we set them here


//        mOpMode.telemetry.addData("Yaw:", desiredYaw);
//        mOpMode.telemetry.addData("Pitch:", desiredPitch);


        yaw.setPosition(desiredYaw);
        pitch.setPosition(desiredPitch);


        if (mHelix.gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .3)
        {
            mGripState = GripState.OPEN;
            mHelix.krakenEye.deployed = false;
            mHelix.krakenEye.hasSample = false;
        } else if(!mHelix.krakenEye.deployed)
        {
            mGripState = GripState.CLOSED;
        }


        if (mGripState == GripState.OPEN) {
            grip.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
            mHelix.krakenEye.hasSample = false;
        } else {
            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
        }
    }

    //set mGripState and set servo accordingly, if its open kraken no haves sample
//    public void SetClawGripState(GripState state){
//        mGripState = state;
//        if (mGripState == GripState.OPEN) {
//            grip.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
//            mHelix.krakenEye.hasSample = false;
//        } else {
//            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
//        }
//    }


    //presets
    public void ChangeClawPositionTo(ClawState newClawState) {
        switch (newClawState) {
            case HOME:
                yaw.setPosition(HelixConstants.YAW_HOME);
                pitch.setPosition(HelixConstants.PITCH_HOME);
                break;
            case HANG:
                yaw.setPosition(HelixConstants.YAW_HANG);
                pitch.setPosition(HelixConstants.PITCH_HANG);
                break;
            case BASKET:

                break;
            case SUB:
                yaw.setPosition(HelixConstants.YAW_SUB);
                pitch.setPosition(HelixConstants.PITCH_SUB);
                break;
        }
    }
}
