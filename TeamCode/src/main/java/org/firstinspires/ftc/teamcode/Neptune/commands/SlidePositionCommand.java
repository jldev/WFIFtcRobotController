package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

public class SlidePositionCommand extends CommandBase {
    private final SlidesSubsystem slides;
    private final SlidesSubsystem.SlidesPosition position;

    public SlidePositionCommand(SlidesSubsystem slides, SlidesSubsystem.SlidesPosition position) {
        this.slides = slides;
        this.position = position;

        addRequirements(slides);
    }

    @Override
    public void initialize() { slides.moveToPosition(this.position);}

    @Override
    public void execute() {
       slides.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {

        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !slides.isBusy();
    }
}
