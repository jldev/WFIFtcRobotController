package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

public class DetectAprilTagCommand extends CommandBase {

    private final VisionSubsystem visionSubsystem;
    private boolean tagFound = false;
    private int desiredTag;
    private AprilTagDetection detection;

    public DetectAprilTagCommand(VisionSubsystem visionSubsystem, int desiredTag){
        this.visionSubsystem = visionSubsystem;
        this.desiredTag = desiredTag;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize(){
        this.visionSubsystem.setMode(VisionSubsystem.VisionMode.APRIL_TAG);
    }
    @Override
    public void execute(){
        List<AprilTagDetection> detections = visionSubsystem.getAprilTagDetections();
        for (AprilTagDetection detection : detections) {
            if(detection.id == this.desiredTag){
                this.detection = detection;
                tagFound = true;
            }
        }   // end for() loop
    }
    @Override
    public void end(boolean interrupted) {
        visionSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || tagFound;
    }

    public AprilTagPoseFtc getPoseFromDetection(){
        if (detection != null) {
            return detection.ftcPose;
        }
        return null;
    }
}
