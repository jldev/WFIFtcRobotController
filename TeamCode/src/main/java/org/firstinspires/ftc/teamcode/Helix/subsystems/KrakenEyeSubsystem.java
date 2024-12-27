package org.firstinspires.ftc.teamcode.Helix.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;

import java.util.List;

public class KrakenEyeSubsystem extends SubsystemBase {

    private final Helix mHelix;

    private final CommandOpMode mOpMode;

    private Limelight3A limelight;

    public enum SampleColor{
        RED,
        BLUE,
        YELLOW
    }
    public SampleColor desiredColor = SampleColor.YELLOW;

    // state variables
    public boolean hasSample = false;
    public boolean deployed = false;

    public boolean doYouClaim(LLResultTypes.ColorResult cr) {
        return Math.abs(cr.getTargetXDegrees()) < 5.00 && (cr.getTargetYDegrees() < 0.00);
    }

    public KrakenEyeSubsystem(Helix helix, CommandOpMode commandOpMode, ClawSubsystem claw, Limelight3A limelight_) {
        mHelix = helix;
        mOpMode = commandOpMode;

        limelight = limelight_;
    }

    @Override
    public void periodic() {

//        desiredYaw = (mHelix.gunnerOp.getLeftX() / 2) + .5;
//        desiredPitch = (mHelix.gunnerOp.getLeftY() / 2) + .5;



        //   GUNNER CONTROL


        //  toggle kraken
        if ((mHelix.gunnerOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .3) && !deployed) {
           DeployTheKraken(desiredColor);
        }

        mOpMode.telemetry.addData("desiredColor", desiredColor);

        // change desiredColor
        if(mHelix.gunnerOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
        {
            switch (desiredColor) {
                case YELLOW: desiredColor = SampleColor.RED; break;
                case RED: desiredColor = SampleColor.BLUE; break;
                case BLUE: desiredColor = SampleColor.YELLOW; break;
            }
        }

//        mOpMode.telemetry.addData("krakenDeployed", deployed);
//        mOpMode.telemetry.addData("krakenHasSample",hasSample);




        if (deployed) {
            if(!hasSample)
            {
                mHelix.claw.mGripState = ClawSubsystem.GripState.OPEN;
            }

            LLResult result = mHelix.limelight.getLatestResult();
            if (result != null) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                for (LLResultTypes.ColorResult cr : colorResults) {
                    mOpMode.telemetry.addData("SAMPLE_X", cr.getTargetXDegrees());
                    mOpMode.telemetry.addData("SAMPLE_Y", cr.getTargetYDegrees());
                    mOpMode.telemetry.addData("SAMPLE_Rotation", GetSampleRotation(cr.getTargetCorners()));

                    if(doYouClaim(cr)){
                        mHelix.claw.mGripState = ClawSubsystem.GripState.CLOSED;
                        hasSample = true;
                        RecallTheKraken();
                    }
                }
                // for each result, if kraken "claims" sample (in specified range) we close and recall kraken
                // (shutdowns limelight & krakenDeployed = false)
            }
        }

        mOpMode.telemetry.addData("krakenDeployed", deployed);
        mOpMode.telemetry.addData("krakenHasSample", hasSample);

        mOpMode.telemetry.update();
    }




    // start the limelight with a specified pipeline and open claw
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
        mHelix.claw.mGripState = ClawSubsystem.GripState.OPEN;
        mHelix.limelight.start();
        deployed = true;
    }




    public void RecallTheKraken(){
        mHelix.limelight.shutdown();
        deployed = false;
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

    //set mGripState and seet servo accordingly, if its open kraken no haves sample
//    public void SetClawGripState(GripState state){
//        mGripState = state;
//        if (mGripState == GripState.OPEN) {
//            grip.setPosition(HelixConstants.GRIPPER_OPEN_VALUE);
//            krakenEye.hasSample = false;
//        } else {
//            grip.setPosition(HelixConstants.GRIPPER_CLOSED_VALUE);
//        }
//    }
}
