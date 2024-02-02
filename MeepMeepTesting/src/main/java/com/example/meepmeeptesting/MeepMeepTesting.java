package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.WFIBotBuilder.getAccelerationConstraint;
import static com.example.meepmeeptesting.WFIBotBuilder.getVelocityConstraint;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class MeepMeepTesting {
    public static double MAX_VEL = 74.01437306619975;
    public static double MAX_ACCEL = 74.01437306619975;
    public static double MAX_ANG_VEL = Math.toRadians(302.9079428571428);
    public static double MAX_ANG_ACCEL = Math.toRadians(302.9079428571428);

    public static double TRACK_WIDTH = 14.88; // in

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d robotOffset = new Pose2d(8,8, Math.toRadians(270));

        Pose2d redAUStart = new Pose2d(-12, 62, Math.toRadians(90));
        Pose2d redBDStart = new Pose2d(-12, 62, Math.toRadians(90));
//        Pose2d blueBackStageStart = new Pose2d(-12, -62, Math.toRadians(270));
//        Pose2d blueFrontStageStart = new Pose2d(35, -62, Math.toRadians(270));

        Pose2d start = redAUStart;

        RoadRunnerBotEntity myBot = new WFIBotBuilder(meepMeep)
                .setStartPose(start)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .build();



        Pose2d redCenterBackdrop = new Pose2d(-48, 36, Math.toRadians(0)); //complete
        Pose2d redLeftBackdrop = new Pose2d(-48, 30, Math.toRadians(0)); //complete
        Pose2d redRightBackdrop = new Pose2d(-48, 42, Math.toRadians(0)); //complete

//        Pose2d blueCenterBackdrop = new Pose2d(-50, -36, Math.toRadians(180));
//        Pose2d blueLeftBackdrop = new Pose2d(-40, -48, Math.toRadians(180)); //this is set
//        Pose2d blueRightBackdrop = new Pose2d(-40, -30, Math.toRadians(180));

        Pose2d redBDCenterSpike =new Pose2d(-24, 24, Math.toRadians(180)); //complete
        Pose2d redBDLeftSpike = new Pose2d(-12, 32, Math.toRadians(180)); //complete
        Pose2d redBDRightSpike = new Pose2d(-36, 32, Math.toRadians(180)); //complete? (does 180 over 12 inches)
        Pose2d redAUCenterSpike =new Pose2d(-12, 32, Math.toRadians(90));
        Pose2d redAULeftSpike = new Pose2d(-7, 34, Math.toRadians(135));
        Pose2d redAURightSpike = new Pose2d(-18, 34, Math.toRadians(45));

        Pose2d redAUIn = new Pose2d(-12, -22, Math.toRadians(180));
        Pose2d redAUOut = new Pose2d(-12, 12, Math.toRadians(180));
        Pose2d redBDIn = new Pose2d(-12, 10, Math.toRadians(180));
        Pose2d redBDOut = new Pose2d(-36, -10, Math.toRadians(180));

//        Pose2d blueBackStageCenterSpike =new Pose2d(-12, -32, Math.toRadians(90+180));
//        Pose2d blueBackStageLeftSpike = new Pose2d(-18, -34, Math.toRadians(135+180));
//        Pose2d blueBackStageRightSpike = new Pose2d(-7, -34, Math.toRadians(45+180));
//
//        Pose2d blueFrontStageCenterSpike =new Pose2d(36, -24, Math.toRadians(90));
//        Pose2d blueFrontStageLeftSpike = new Pose2d(32, -29, Math.toRadians(180)); //this is set
//        Pose2d blueFrontStageRightSpike = new Pose2d(46, -29, Math.toRadians(0));


        Pose2d backdrop = redRightBackdrop;
        Pose2d spike = redBDCenterSpike;
        Pose2d inOut = redBDIn;

//        backdrop.minus(robotOffset);
//        spike.minus(robotOffset);

        TrajectoryVelocityConstraint SLOW_VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL/2, MAX_ANG_VEL/2, TRACK_WIDTH);
        TrajectoryAccelerationConstraint SLOW_ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL/2);

//        Trajectory init = new TrajectoryBuilder(start, true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
//                .back(20)
//                .build();

        Trajectory pixelTraj = new TrajectoryBuilder(start, true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToLinearHeading(spike)
                .build();

//        Trajectory back3 = new TrajectoryBuilder(pixelTraj.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
//                .back(3)
//                .build();

//        Trajectory strafeRight19 = new TrajectoryBuilder(back3.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
//                .strafeRight(19)
//                .build();

//        Trajectory backStageStagingIn = new TrajectoryBuilder(pixelTraj.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
//                .lineToSplineHeading(inOut)
//                .build();

        Trajectory backdropTraj = new TrajectoryBuilder(spike, false, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToLinearHeading(backdrop)
                .build();

        Trajectory backStageStagingOut = new TrajectoryBuilder(backdropTraj.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(-36, -10, Math.toRadians(180)))
                .build();

        Trajectory leftPixelStack = new TrajectoryBuilder(backStageStagingOut.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(59, -10, Math.toRadians(180)))
                .build();



        myBot.followTrajectorySequence(
                myBot.getDrive().trajectorySequenceBuilder(start)
//                        .addTrajectory(init)
                        .addTrajectory(pixelTraj)
//                        .addTrajectory(back3)
//                        .addTrajectory(strafeRight19)
//                        .addTrajectory(backStageStagingIn)
                        .addTrajectory(backdropTraj)
//                        .addTrajectory(backStageStagingOut)
//                        .addTrajectory(leftPixelStack)
                        .build()
        );





        try {
            File pathToFile = new File(".\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\field-2023-official.png");
            Image image = ImageIO.read(pathToFile);
            meepMeep.setBackground(image)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        } catch (IOException ex) {
            ex.printStackTrace();
        }


    }
}