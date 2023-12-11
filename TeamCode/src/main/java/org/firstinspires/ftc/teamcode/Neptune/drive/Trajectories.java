package org.firstinspires.ftc.teamcode.Neptune.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

public class Trajectories {

    public enum PropPlacement {
        LEFT,
        RIGHT,
        CENTER
    }

    private MecanumDriveSubsystem mDrive;
    private Pose2d mStartPosition;

    public Trajectories(MecanumDriveSubsystem drive, Pose2d startPosition){
        this.mDrive = drive;
        this.mStartPosition = startPosition;
    }

    public Trajectory getPlacePixelTrajectory(PropPlacement placement){
        //     The position we go to after locating the team prop, lining up to go to the correct spike mark
        Pose2d initialSpike = new Pose2d(-12, 46, Math.toRadians(90));

        //     The left, center, and right spike mark locations
        Pose2d leftSpike = new Pose2d(0, 35, Math.toRadians(135));
        Pose2d centerSpike = new Pose2d(-12, 32, Math.toRadians(90));
        Pose2d rightSpike = new Pose2d(-12, 29, Math.toRadians(45));

        switch (placement){
            case LEFT:
                return mDrive.trajectoryBuilder(mStartPosition)
                        .splineTo(initialSpike.vec(), initialSpike.getHeading())
                        .splineTo(leftSpike.vec(), leftSpike.getHeading())
                        .build();
            case RIGHT:
                return mDrive.trajectoryBuilder(mStartPosition)
                        .splineTo(initialSpike.vec(), initialSpike.getHeading())
                        .splineTo(rightSpike.vec(), rightSpike.getHeading())
                        .build();
            case CENTER:
                return mDrive.trajectoryBuilder(mStartPosition)
                        .splineTo(initialSpike.vec(), initialSpike.getHeading())
                        .splineTo(centerSpike.vec(), centerSpike.getHeading())
                        .build();
            default:
                break;


        }
        return mDrive.trajectoryBuilder(mStartPosition)
                .splineTo(initialSpike.vec(), initialSpike.getHeading())
                .splineTo(leftSpike.vec(), leftSpike.getHeading())
                .build();
    }

}
