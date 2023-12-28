package org.firstinspires.ftc.teamcode.Neptune.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

public class Trajectories {



    public enum PropPlacement {
        LEFT,
        RIGHT,
        CENTER
    }

    public enum StackPos {
        LEFTSTACK,
        CENTERSTACK,
        RIGHTSTACK

    }


    private MecanumDriveSubsystem mDrive;
    private Pose2d mStartPosition;
    private Neptune neptune;

    public Trajectories(Neptune neptune){
        this.mDrive = neptune.drive;
        this.mStartPosition = neptune.startPos;
        this.neptune = neptune;
    }

    public Trajectory getPlacePixelTrajectory(PropPlacement placement){
        //     The position we go to after locating the team prop, lining up to go to the correct spike mark
        Pose2d initialSpike = new Pose2d(-12, 46, Math.toRadians(90));
        //     The left, center, and right spike mark locations
        Pose2d leftSpike = new Pose2d(0, 35, Math.toRadians(135));
        Pose2d centerSpike = new Pose2d(-12, 32, Math.toRadians(90));
        Pose2d rightSpike = new Pose2d(-12, 29, Math.toRadians(45));

        if (this.neptune.allianceColor == Neptune.AllianceColor.RED) {
            if (this.neptune.fieldPos == Neptune.FieldPos.LEFT) {
                 leftSpike = new Pose2d(48, 35, Math.toRadians(135));
                 centerSpike = new Pose2d(36, 32, Math.toRadians(90));
                 rightSpike = new Pose2d(36, 29, Math.toRadians(45));
            } else  {
                //this is right
                leftSpike = new Pose2d(0, 35, Math.toRadians(135));
                centerSpike = new Pose2d(-12, 32, Math.toRadians(90));
                rightSpike = new Pose2d(-12, 29, Math.toRadians(45));
            }
        } else {
            //this is alliance blue
            if (this.neptune.fieldPos == Neptune.FieldPos.LEFT) {
                //needs to be added eventually
            } else {
                //this is left


            }
        }
        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(leftSpike)
                        .build();
        switch (placement){
            case LEFT:
                traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(leftSpike)
                        .build();
                break;
            case RIGHT:
                traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(rightSpike)
                        .build();
                break;
            case CENTER:
                traj = mDrive.trajectoryBuilder(mStartPosition, true)
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

    public Trajectory getPixelFromStack (StackPos stackPos){

        Pose2d leftStack = new Pose2d(51, 36, Math.toRadians(0));
        Pose2d centerStack = new Pose2d(51, 24, Math.toRadians(0));
        Pose2d rightStack = new Pose2d(54, 12, Math.toRadians(0));

        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, true)
                .splineToConstantHeading(leftStack.vec(),leftStack.getHeading())
                .build();


        switch (stackPos){
            case LEFTSTACK:
                traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(leftStack)
//                        .splineToConstantHeading(leftStack.vec(),leftStack.getHeading())
                        .build();
                break;
            case CENTERSTACK:
                traj = mDrive.trajectoryBuilder(mStartPosition, true)
                        .lineToSplineHeading(centerStack)
//                        .splineToConstantHeading(centerStack.vec(),centerStack.getHeading())
                        .build();
                break;
            case RIGHTSTACK:
                 traj = mDrive.trajectoryBuilder(mStartPosition, true)
                         .lineToSplineHeading(rightStack)
//                        .splineToConstantHeading(rightStack.vec(),rightStack.getHeading())
                        .build();
                break;
            default:
                break;


        }


        mStartPosition = traj.end();
        return traj;
    }

    public Trajectory getStageTrajectory() {
        Pose2d stageIn = new Pose2d(0, 0, Math.toRadians(0));

        return getTrajectory(stageIn);
    }

    public Trajectory getTrajectory(Pose2d pose2d){
        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition)
                .lineToSplineHeading(pose2d)
                .build();

        mStartPosition = traj.end();
        return traj;
    }


}
