package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
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
@Disabled
public class rr_testtt extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-56, -12, 0);

        drive.setPoseEstimate(startPose);

         TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
            .forward(15)
//                 .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))
                         .build();





//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .forward(15)
//
//                .build();

//        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
//                .turn(Math.toRadians(45)) // Turns 45 degrees counter-clockwise
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
//                .forward(-)
//                .build();

//        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(-36, 0), Math.toRadians(0))
//                .build();



//        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
//                .turn(Math.toRadians(30)) // Turns 45 degrees counter-clockwise
//                .build();




        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(traj1);
            sleep(10000);
//
//            drive.followTrajectorySequence(ts);
//            drive.followTrajectory(traj3);

        }
    }
}