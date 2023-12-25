package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.ObjectDetectionSubsystem;

public class DetectPawnCommand extends CommandBase {

    private final ObjectDetectionSubsystem objectDetectionSubsystem;

    public DetectPawnCommand(ObjectDetectionSubsystem ods){
        this.objectDetectionSubsystem = ods;

        addRequirements(objectDetectionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        objectDetectionSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !objectDetectionSubsystem.getRecognitions().isEmpty();
    }

    public Trajectories.PropPlacement getPropLocation(){
        if (!objectDetectionSubsystem.getRecognitions().isEmpty()){
            // this is making an assumption of only one detection
            double[] location = objectDetectionSubsystem.getCenterOfRecognition(objectDetectionSubsystem.getRecognitions().get(0));
            if (location[0] > 400) { // - && y > 200 - this may not work
                return Trajectories.PropPlacement.RIGHT;
            } else if (location[0] < 100) {
                return Trajectories.PropPlacement.LEFT;
            } else {
                return Trajectories.PropPlacement.CENTER;
            }
        }
        return Trajectories.PropPlacement.LEFT;
    }
}
