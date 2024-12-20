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
    private GripState mGripState;

    private double desiredYaw;
    private double desiredPitch;

    private Limelight3A limelight;

    public enum SampleColor{
        RED,
        BLUE,
        YELLOW
    }
    public class KrakenEye {
        // state variables
        public boolean hasSample = false;
        public boolean deployed = false;

        public boolean doYouClaim(LLResultTypes.ColorResult cr) {
            return Math.abs(cr.getTargetXDegrees()) < 5.00 && (cr.getTargetYDegrees() < 0.00);
        }
    }

    private final KrakenEye krakenEye = new KrakenEye();

    public ClawSubsystem(Helix helix, CommandOpMode commandOpMode, Servo yaw_1, Servo pitch_2, Servo grip_3, Limelight3A limelight_) {
        mHelix = helix;
        mOpMode = commandOpMode;

        yaw = yaw_1;
        pitch = pitch_2;
        grip = grip_3;

        limelight = limelight_;

        if(helix.mOpModeType == Helix.OpModeType.AUTO)
        {
            yaw.setPosition(HelixConstants.CLAW_YAW_INIT);
            pitch.setPosition(HelixConstants.CLAW_PITCH_INIT);
            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
        }
    }

    @Override
    public void periodic() {

//        desiredYaw = (mHelix.gunnerOp.getLeftX() / 2) + .5;
//        desiredPitch = (mHelix.gunnerOp.getLeftY() / 2) + .5;


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

        if ((mHelix.gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .3)) {
            SetClawGripState(GripState.OPEN);
        } else if (mHelix.mOpModeType == Helix.OpModeType.TELEOP && !krakenEye.deployed) {
            SetClawGripState(GripState.CLOSED);
        }


        if ((mHelix.gunnerOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .3) && !krakenEye.deployed) {
           DeployTheKraken(SampleColor.RED);
        }


        // After we get our positions from manual or auto - we set them here


//        mOpMode.telemetry.addData("Yaw:", desiredYaw);
//        mOpMode.telemetry.addData("Pitch:", desiredPitch);


        yaw.setPosition(desiredYaw);
        pitch.setPosition(desiredPitch);

        if (krakenEye.deployed) {
            if(!krakenEye.hasSample)
            {
                SetClawGripState(GripState.OPEN);
            }

            LLResult result = mHelix.limelight.getLatestResult();
            if (result != null) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                for (LLResultTypes.ColorResult cr : colorResults) {
                    mOpMode.telemetry.addData("SAMPLE_X", cr.getTargetXDegrees());
                    mOpMode.telemetry.addData("SAMPLE_Y", cr.getTargetYDegrees());
                    mOpMode.telemetry.addData("SAMPLE_Rotation", GetSampleRotation(cr.getTargetCorners()));
                    mOpMode.telemetry.update();

                    if(krakenEye.doYouClaim(cr)){
                        SetClawGripState(GripState.CLOSED);
                        krakenEye.hasSample = true;
                        RecallTheKraken();
                    }
                }
            }
        }
    }

    public void DeployTheKraken(SampleColor color){
        switch(color){
            case RED:
                mHelix.limelight.pipelineSwitch(0);
                break;
            case BLUE:
                mHelix.limelight.pipelineSwitch(1);
                break;
            case YELLOW:
                mHelix.limelight.pipelineSwitch(2);
                break;
        }
        mHelix.limelight.start();
        krakenEye.deployed = true;
    }

    public void RecallTheKraken(){
        mHelix.limelight.shutdown();
        krakenEye.deployed = false;
    }
    private double GetSampleRotation(List<List<Double>> corners){
        if(corners.size() < 2){
            return 0.0;
        }
        List<Double> bottomLeft = corners.get(0);
        List<Double> topLeft = corners.get(1);
        double adjacentSide = topLeft.get(1) - bottomLeft.get(1);
        double oppositeSize = topLeft.get(0) - bottomLeft.get(0);
        double angleRadians = Math.atan(oppositeSize/adjacentSide);
        return Math.toDegrees(angleRadians);
    }

    public void SetClawGripState(GripState state){
        mGripState = state;
        if (mGripState == GripState.OPEN) {
            grip.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
        } else {
            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
        }
    }

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

        if (mGripState == GripState.OPEN) {
            grip.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
        } else {
            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
        }
    }
}
