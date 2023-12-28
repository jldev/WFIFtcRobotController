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
        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition)
                        .lineToSplineHeading(leftSpike)
                        .build();
        switch (placement){
            case LEFT:
                traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(leftSpike)
                        .build();
                break;
            case RIGHT:
                traj = mDrive.trajectoryBuilder(mStartPosition)
                        .lineToSplineHeading(rightSpike)
                        .build();
                break;
            case CENTER:
                traj = mDrive.trajectoryBuilder(mStartPosition)
                        .lineToSplineHeading(centerSpike)
                        .build();
                break;
            default:
                break;


        }
        mStartPosition = traj.end();
        return traj;
    }

    public Trajectory getBackdropTrajectory(PropPlacement placement){
        //     The position we go to after locating the team prop, lining up to go to the correct spike mark

        //     The left, center, and right spike mark locations
        Pose2d leftBackdrop = new Pose2d(-50, 28, Math.toRadians(0));
        Pose2d centerBackdrop = new Pose2d(-50, 35, Math.toRadians(0));
        Pose2d rightBackdrop = new Pose2d(-50, 42, Math.toRadians(0));
        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition)
                .lineToSplineHeading(leftBackdrop)
                .build();

        switch (placement){
            case LEFT:
               traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(leftBackdrop)
                        .build();
               break;
            case CENTER:
                traj = mDrive.trajectoryBuilder(mStartPosition)
                        .lineToSplineHeading(centerBackdrop)
                        .build();
                break;
            case RIGHT:
                traj = mDrive.trajectoryBuilder(mStartPosition)
                        .lineToSplineHeading(rightBackdrop)
                        .build();
                break;
            default:
                break;


        }
        mStartPosition = traj.end();
        return traj;
    }

    public Trajectory getPixelFromStack (){
        Pose2d stageIn = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d rightStack = new Pose2d(60, 12, Math.toRadians(0));

        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition)
                .splineToConstantHeading(stageIn.vec(),stageIn.getHeading())
                .splineToConstantHeading(rightStack.vec(),rightStack.getHeading())
                .build();

        mStartPosition = traj.end();
        return traj;
    }


}
