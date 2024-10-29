package org.firstinspires.ftc.teamcode.Helix.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceFollowerCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final TrajectorySequence trajectorySequence;

    public TrajectorySequenceFollowerCommand(MecanumDriveSubsystem drive, TrajectorySequence trajectorySequence) {
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(this.trajectorySequence);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}
