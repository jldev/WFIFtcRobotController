package org.firstinspires.ftc.teamcode.Neptune.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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




    private MecanumDriveSubsystem mDrive;
    private Pose2d mStartPosition;
    private Neptune neptune;

    public Trajectories(Neptune neptune){
        this.mDrive = neptune.drive;
        this.mStartPosition = neptune.startPos;
        this.neptune = neptune;
    }

    /*
        If the field is symmetric, you can use this to translate from RED to BLUE
        we will rotate about x 180 degrees, using the right hand rule this would turn our field upside down,
        but since we don't care about the Z(UP) direction this will be fine, the following matrix
        rotates us around x
        1	0	0
        0	cos(a)	-sin(a)
        0	sin(a)	cos(a)
        for 180 rotation aournd the X axis we get
        1   0   0
        0   -1  0
        0   0   -1
        we dont care about the Z so the 2x2 matrix becomes
        1    0
        0   -1
        given input point x,y
        new_x = x*1 - 0 = x
        new_y = x*0 -y = -y
        so all that to say just multiply the y by -1
        all the headings are +180 or PI in radians
    */

    public static Pose2d translatePosePositionToBlue(Pose2d p){
        return new Pose2d(p.getX(),-p.getY(), p.getHeading()+Math.PI);
    }
    public Trajectory getPlacePixelTrajectory(PropPlacement placement){
        //     The position we go to after locating the team prop, lining up to go to the correct spike mark
        Pose2d initialSpike;
        //     The left, center, and right spike mark locations
        Pose2d leftSpike;
        Pose2d centerSpike;
        Pose2d rightSpike;

        // these are for the red side, if we are blue we translate them below
        if (this.neptune.fieldPos == Neptune.FieldPos.LEFT) {
             initialSpike = new Pose2d(-12, 46, Math.toRadians(90));
             leftSpike = new Pose2d(48, 35, Math.toRadians(135));
             centerSpike = new Pose2d(36, 32, Math.toRadians(90));
             rightSpike = new Pose2d(36, 29, Math.toRadians(45));

        } else  {
            //this is right
            initialSpike = new Pose2d(-12, 46, Math.toRadians(90));
            leftSpike = new Pose2d(0, 35, Math.toRadians(135));
            centerSpike = new Pose2d(-12, 32, Math.toRadians(90));
            rightSpike = new Pose2d(-12, 29, Math.toRadians(45));
        }

        if(this.neptune.allianceColor == Neptune.AllianceColor.BLUE){
            initialSpike = translatePosePositionToBlue(initialSpike);
            leftSpike = translatePosePositionToBlue(leftSpike);
            centerSpike = translatePosePositionToBlue(centerSpike);
            rightSpike = translatePosePositionToBlue(rightSpike);
        }

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
            leftBackdrop = translatePosePositionToBlue(leftBackdrop);
            centerBackdrop = translatePosePositionToBlue(centerBackdrop);
            rightBackdrop = translatePosePositionToBlue(rightBackdrop);
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
