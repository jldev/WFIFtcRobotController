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

        //Start locations
        Pose2d AUStart = new Pose2d(-12, 62, Math.toRadians(90));
        Pose2d BDStart = new Pose2d(-12, 62, Math.toRadians(90));

        //Spike locations for Backdrop side
        Pose2d BDCenterSpike =new Pose2d(-24, 24, Math.toRadians(180)); //complete
        Pose2d BDLeftSpike = new Pose2d(-12, 32, Math.toRadians(180)); //complete
        Pose2d BDRightSpike = new Pose2d(-36, 32, Math.toRadians(180)); //complete
        //Spike locations for Audience side
        Pose2d AUCenterSpike =new Pose2d(-12, 32, Math.toRadians(90));
        Pose2d AULeftSpike = new Pose2d(-7, 34, Math.toRadians(135));
        Pose2d AURightSpike = new Pose2d(-18, 34, Math.toRadians(45));

        //Backdrop locations
        Pose2d CenterBackdrop = new Pose2d(-48, 36, Math.toRadians(0)); //complete
        Pose2d LeftBackdrop = new Pose2d(-48, 30, Math.toRadians(0)); //complete
        Pose2d RightBackdrop = new Pose2d(-48, 42, Math.toRadians(0)); //complete

        //Stage midpoint locations
        Pose2d AUIn = new Pose2d(48, 12, Math.toRadians(0)); // complete
        Pose2d AUOut = new Pose2d(48, 60, Math.toRadians(0));
        Pose2d BDIn = new Pose2d(-24, 12, Math.toRadians(0)); //complete
        Pose2d BDOut = new Pose2d(-24, 60, Math.toRadians(0));

        //Base points, where we need to go
        Pose2d startOrigin = BDStart;
          //Both left or right
        Pose2d spikeOrigin = BDLeftSpike;
        Pose2d backdropOrigin = LeftBackdrop;
          //Both in or out
        Pose2d AUInOutOrigin = AUOut;
        Pose2d BDInOutOrigin = BDOut;

        //If we're on the blue or red alliance
        int redBlue = 1; //  red = 1 | blue = -1
        //Translates
        if(redBlue == -1)
        {
            //Flips left & right if we're on blue
            if(spikeOrigin == BDRightSpike)
            {spikeOrigin = BDLeftSpike; backdropOrigin = LeftBackdrop;}
            else {spikeOrigin = BDRightSpike; backdropOrigin = RightBackdrop;}
        }
        Pose2d start = new Pose2d(startOrigin.getX(), redBlue * startOrigin.getY(), startOrigin.getHeading());
        Pose2d spike = new Pose2d(spikeOrigin.getX(), redBlue * spikeOrigin.getY(), spikeOrigin.getHeading());
        Pose2d backdrop = new Pose2d(backdropOrigin.getX(), redBlue * backdropOrigin.getY(), backdropOrigin.getHeading());
        Pose2d AUInOut = new Pose2d(AUInOutOrigin.getX(), redBlue * AUInOutOrigin.getY(), AUInOutOrigin.getHeading());
        Pose2d BDInOut = new Pose2d(BDInOutOrigin.getX(), redBlue * BDInOutOrigin.getY(), BDInOutOrigin.getHeading());


        RoadRunnerBotEntity myBot = new WFIBotBuilder(meepMeep)
                .setStartPose(start)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .build();


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

        Trajectory BDInOutTraj = new TrajectoryBuilder(backdrop, false, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToLinearHeading(BDInOut)
                .build();

        Trajectory AUInOutTraj = new TrajectoryBuilder(BDInOut, false, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToLinearHeading(AUInOut)
                .build();

//        Trajectory backStageStagingOut = new TrajectoryBuilder(backdropTraj.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
//                .lineToSplineHeading(new Pose2d(-36, -10, Math.toRadians(180)))
//                .build();

//        Trajectory leftPixelStack = new TrajectoryBuilder(backStageStagingOut.end(), true, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
//                .lineToSplineHeading(new Pose2d(59, -10, Math.toRadians(180)))
//                .build();



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
                        .addTrajectory(BDInOutTraj)
                        .addTrajectory(AUInOutTraj)
                        .build()
        );





        try {
            File pathToFile = new File("C:\\Users\\Josh\\code\\WFIFtcRobotController\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\field-2023-official.png");
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