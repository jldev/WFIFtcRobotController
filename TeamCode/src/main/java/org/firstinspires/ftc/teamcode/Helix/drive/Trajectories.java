package org.firstinspires.ftc.teamcode.Helix.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Helix.Helix;

public class Trajectories {


    private static final boolean TRAJECTORY_SPEED_SLOW = true;

    //Helix Test Drive Waypoints (waypoints just sounds cool)


    public Pose2d waypoint0 = new Pose2d(0, 0, Math.toRadians(0));
    public Pose2d waypoint1 = new Pose2d(24, 0, Math.toRadians(90));
    public Pose2d waypoint2 = new Pose2d(0, 24, Math.toRadians(180));
    private Helix mHelix;



    public Trajectories(Helix helix){
        helix.currentPos = new Pose2d(0, 0, Math.toRadians(0)); // where we are currently
        mHelix = helix;
    }


    public Trajectory getTrajectory(Pose2d pose2d){
        Trajectory traj = mHelix.drive.trajectoryBuilder(mHelix.currentPos, false, TRAJECTORY_SPEED_SLOW)
                .lineToSplineHeading(pose2d)
                .build();

        mHelix.currentPos = traj.end();
        return traj;
    }

}
