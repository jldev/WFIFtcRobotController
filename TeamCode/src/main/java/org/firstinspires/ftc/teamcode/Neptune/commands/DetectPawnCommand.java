package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;

public class DetectPawnCommand extends CommandBase {

    private final VisionSubsystem visionSubsystem;

    public DetectPawnCommand(VisionSubsystem visionSubsystem){
        this.visionSubsystem = visionSubsystem;
        this.visionSubsystem.setMode(VisionSubsystem.VisionMode.TENSORFLOW);
        addRequirements(visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        visionSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !visionSubsystem.getRecognitions().isEmpty();
    }

    public Trajectories.PropPlacement getPropLocation(){
        if (!visionSubsystem.getRecognitions().isEmpty()){
            // this is making an assumption of only one detection
            double[] location = visionSubsystem.getCenterOfRecognition(visionSubsystem.getRecognitions().get(0));
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
