package org.firstinspires.ftc.teamcode.Neptune.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class Trajectories {


    private static final boolean TRAJECTORY_SPEED_SLOW = true;

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
    //Start locations
    public static Pose2d AUStart = new Pose2d(36, 62, Math.toRadians(90)); //complete
    public static Pose2d BDStart = new Pose2d(-12, 62, Math.toRadians(90)); //complete

    //Spike locations for Backdrop side
    Pose2d BDCenterSpike =new Pose2d(-24, 24, Math.toRadians(180)); //complete
    Pose2d BDLeftSpike = new Pose2d(-12, 32, Math.toRadians(180)); //complete
    Pose2d BDRightSpike = new Pose2d(-36, 32, Math.toRadians(180)); //complete

    //Spike locations for Audience side
    Pose2d AUCenterSpike =new Pose2d(48, 24, Math.toRadians(0)); //complete
    Pose2d AULeftSpike = new Pose2d(57, 32, Math.toRadians(0)); //complete
    Pose2d AURightSpike = new Pose2d(36, 32, Math.toRadians(0)); //complete

    //Backdrop locations
    Pose2d CenterBackdrop = new Pose2d(-48, 36, Math.toRadians(0)); //complete
    Pose2d LeftBackdrop = new Pose2d(-48, 30, Math.toRadians(0)); //complete
    Pose2d RightBackdrop = new Pose2d(-48, 42, Math.toRadians(0)); //complete

    //Stack locations
    Pose2d OuterStack = new Pose2d(60, 36, Math.toRadians(0));  //complete
    Pose2d CenterStack = new Pose2d(60, 24, Math.toRadians(0)); //complete
    Pose2d InnerStack = new Pose2d(60, 12, Math.toRadians(0)); //complete

    //Stage midpoint locations
    Pose2d AUIn = new Pose2d(48, 12, Math.toRadians(0)); // complete
    Pose2d AUOut = new Pose2d(48, 60, Math.toRadians(0)); //complete
    Pose2d BDIn = new Pose2d(-24, 12, Math.toRadians(0)); //complete
    Pose2d BDOut = new Pose2d(-24, 60, Math.toRadians(0)); //complete

    //Both left or right
    Pose2d spikeOrigin = AUCenterSpike;
    Pose2d backdropOrigin = CenterBackdrop;
    Pose2d stackOrigin = InnerStack;
    //Both in or out
    Pose2d AUInOutOrigin = AUIn;
    Pose2d BDInOutOrigin = BDIn;

    public Pose2d spike;
    public Pose2d stack;
    public Pose2d backdrop;
    public Pose2d AUInOut;
    public Pose2d BDInOut;

    private MecanumDriveSubsystem mDrive;
    public Pose2d mStartPosition;
    private Neptune neptune;

    //If we're on the blue or red alliance
    int redBlue = 1; //  red = 1 | blue = -1

    public Trajectories(Neptune neptune){
        this.mDrive = neptune.drive;
        this.mStartPosition = neptune.startPos;
        this.neptune = neptune;

        if (this.neptune.allianceColor == Neptune.AllianceColor.BLUE){
            redBlue = -1;
        }

    }

    public void setupTrajectories(PropPlacement propLocation){
        switch(propLocation){
            case LEFT:
                if (neptune.fieldPos == Neptune.FieldPos.BD)
                    spikeOrigin = BDLeftSpike;
                else
                    spikeOrigin = AULeftSpike;
                break;
            case RIGHT:
                if (neptune.fieldPos == Neptune.FieldPos.BD)
                    spikeOrigin = BDRightSpike;
                else
                    spikeOrigin = AURightSpike;
                break;
            case CENTER:
                if (neptune.fieldPos == Neptune.FieldPos.BD)
                    spikeOrigin = BDCenterSpike;
                else
                    spikeOrigin = AUCenterSpike;
                break;
        }

        //Translates
        if(this.neptune.allianceColor == Neptune.AllianceColor.BLUE)
        {
            //Flips left & right of the spike mark, stack, and backdrop if we're on blue
            if(spikeOrigin == BDRightSpike)
            {spikeOrigin = BDLeftSpike; backdropOrigin = LeftBackdrop; stackOrigin = OuterStack;}   //flips right to left
            else {spikeOrigin = BDRightSpike; backdropOrigin = RightBackdrop; stackOrigin = InnerStack;}   //flips left to right
        }

        spike = new Pose2d(spikeOrigin.getX(), redBlue * spikeOrigin.getY(), spikeOrigin.getHeading());
        stack = new Pose2d(stackOrigin.getX(), redBlue * stackOrigin.getY(), stackOrigin.getHeading());
        backdrop = new Pose2d(backdropOrigin.getX(), redBlue * backdropOrigin.getY(), backdropOrigin.getHeading());
        AUInOut = new Pose2d(AUInOutOrigin.getX(), redBlue * AUInOutOrigin.getY(), AUInOutOrigin.getHeading());
        BDInOut = new Pose2d(BDInOutOrigin.getX(), redBlue * BDInOutOrigin.getY(), BDInOutOrigin.getHeading());
    }

/*
    public Trajectory getPlacePixelTrajectory(PropPlacement placement){
        //     The position we go to after locating the team prop, lining up to go to the correct spike mark
        Pose2d initialSpike;
        //     The left, center, and right spike mark locations
        Pose2d leftSpike;
        Pose2d centerSpike;
        Pose2d rightSpike;

        // these are for the red side, if we are blue we translate them below
        if (this.neptune.fieldPos == Neptune.FieldPos.AU) {
             initialSpike = new Pose2d(-12, 46, Math.toRadians(90));
             leftSpike = new Pose2d(43, 36, Math.toRadians(135));
             centerSpike = new Pose2d(36, 36, Math.toRadians(90));
             rightSpike = new Pose2d(38, 38, Math.toRadians(45));
             //left
            if (this.neptune.allianceColor == Neptune.AllianceColor.BLUE){
                initialSpike = new Pose2d(-12, -46, Math.toRadians(270));
                rightSpike = new Pose2d(0, -35, Math.toRadians(315));
                centerSpike = new Pose2d(-12, -32, Math.toRadians(270));
                leftSpike = new Pose2d(-12, -29, Math.toRadians(225));
            }

        } else  {
            //this is right
            initialSpike = new Pose2d(-12, 46, Math.toRadians(90));
            leftSpike = new Pose2d(0, 35, Math.toRadians(135));
            centerSpike = new Pose2d(-12, 32, Math.toRadians(90));
            rightSpike = new Pose2d(-12, 29, Math.toRadians(45));
            if (this.neptune.allianceColor == Neptune.AllianceColor.BLUE){
                initialSpike = new Pose2d(-14, -48, Math.toRadians(270));
                rightSpike = new Pose2d(45, -36, Math.toRadians(315));
                centerSpike = new Pose2d(35, -33, Math.toRadians(270));
                leftSpike = new Pose2d(34, -34, Math.toRadians(225));
            }
        }


//        if(this.neptune.allianceColor == Neptune.AllianceColor.BLUE){
//            initialSpike = translatePosePositionToBlue(initialSpike);
//            leftSpike = translatePosePositionToBlue(leftSpike);
//            centerSpike = translatePosePositionToBlue(centerSpike);
//            rightSpike = translatePosePositionToBlue(rightSpike);
//
//            Pose2d temp = new Pose2d(leftSpike.getX(), leftSpike.getY(), leftSpike.getHeading());
//            leftSpike = rightSpike;
//            rightSpike = temp;
//
//        }


        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(leftSpike)
                        .build();
        switch (placement){
            case LEFT:
                traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(leftSpike)
                        .build();
                break;
            case RIGHT:
                traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(rightSpike)
                        .build();
                break;
            case CENTER:
                traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
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

        //     The left, center, and right spike mark locations for RED
        Pose2d leftBackdrop = new Pose2d(-40, 34, Math.toRadians(0));
        Pose2d centerBackdrop = new Pose2d(-40, 41, Math.toRadians(0));
        Pose2d rightBackdrop = new Pose2d(-40, 48, Math.toRadians(0));

        if(this.neptune.allianceColor == Neptune.AllianceColor.BLUE){
            rightBackdrop = translatePosePositionToBlue(leftBackdrop);
            centerBackdrop = translatePosePositionToBlue(centerBackdrop);
            leftBackdrop = translatePosePositionToBlue(rightBackdrop);
        }

        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, false, TRAJECTORY_SPEED_SLOW)
                .lineToSplineHeading(leftBackdrop)
                .build();

        switch (placement){
            case LEFT:
               traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(leftBackdrop)
                        .build();
               break;
            case CENTER:
                traj = mDrive.trajectoryBuilder(mStartPosition, false, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(centerBackdrop)
                        .build();
                break;
            case RIGHT:
                traj = mDrive.trajectoryBuilder(mStartPosition, false, TRAJECTORY_SPEED_SLOW)
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

        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                .splineToConstantHeading(leftStack.vec(),leftStack.getHeading())
                .build();


        switch (stackPos){
            case LEFTSTACK:
                traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(leftStack)
//                        .splineToConstantHeading(leftStack.vec(),leftStack.getHeading())
                        .build();
                break;
            case CENTERSTACK:
                traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
                        .lineToSplineHeading(centerStack)
//                        .splineToConstantHeading(centerStack.vec(),centerStack.getHeading())
                        .build();
                break;
            case RIGHTSTACK:
                 traj = mDrive.trajectoryBuilder(mStartPosition, true, TRAJECTORY_SPEED_SLOW)
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
*/
    public Trajectory getTrajectory(Pose2d pose2d){
        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, false, TRAJECTORY_SPEED_SLOW)
                .lineToSplineHeading(pose2d)
                .build();

        mStartPosition = traj.end();
        return traj;
    }

    public Trajectory getTrajectoryForAprilTag(AprilTagPoseFtc pose, int distanceFromTag){

        double  rangeError      = (pose.range - distanceFromTag);
        double  newHeading    = mStartPosition.getHeading() + pose.bearing;
        double  yawError        = pose.yaw;

        Trajectory traj = mDrive.trajectoryBuilder(mStartPosition, false, TRAJECTORY_SPEED_SLOW)
                .splineToConstantHeading(mStartPosition.vec(), newHeading)
                .strafeRight(-yawError)
                .forward(rangeError)
                .build();

        mStartPosition = traj.end();
        return traj;
    }


}
