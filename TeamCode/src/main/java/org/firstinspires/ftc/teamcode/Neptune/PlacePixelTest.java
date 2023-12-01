package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class PlacePixelTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, 62, Math.toRadians(270));

        Pose2d leftSpikeMarkPlacement = new Pose2d(-6.5,40, Math.toRadians(315));
        Pose2d centerSpikeMarkPlacement = new Pose2d(-23, 30, Math.toRadians(315));
        Pose2d centerBackstopPlacement = new Pose2d(-48, 36, Math.toRadians(0));
        Pose2d rightSpikeMarkPlacement = new Pose2d(-17.5,40,Math.toRadians(315));

        drive.setPoseEstimate(startPose);

        //Set these based on the detection of the prop on the spike marks
        Pose2d pixelPlacementTraj = centerSpikeMarkPlacement;
        Pose2d backdropPlacementTraj = centerBackstopPlacement;



        Trajectory placePixelTraj = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(pixelPlacementTraj)

                .build();

        Trajectory driveToBackboardTraj = drive.trajectoryBuilder(pixelPlacementTraj)
                .lineToSplineHeading(backdropPlacementTraj)
                .build();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(placePixelTraj);
        drive.followTrajectory(driveToBackboardTraj);

        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}