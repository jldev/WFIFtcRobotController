package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

public class SlidePositionCommand extends CommandBase {
    private final SlidesSubsystem slides;
    private final SlidesSubsystem.SlidesPosition slidesPosition;

    public SlidePositionCommand(SlidesSubsystem slides, SlidesSubsystem.SlidesPosition slidesPosition) {
        this.slides = slides;
        this.slidesPosition = slidesPosition;

        addRequirements(slides);
    }

    @Override
    public void initialize() {slides.moveToPosition(this.slidesPosition);}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !slides.isBusy();
    }
}
