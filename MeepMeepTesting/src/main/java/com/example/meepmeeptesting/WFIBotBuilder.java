package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class WFIBotBuilder {

    private final MeepMeep meepMeep;
    private Pose2d startPose;

    private Constraints constraints;
    private final double opacity = 0.9;
    private double width = 17;
    private double height = 17;


    public WFIBotBuilder(MeepMeep meepMeep){
        this.meepMeep = meepMeep;
    }
    public WFIBotBuilder setStartPose(Pose2d start) {
        this.startPose = start;
        return this;
    }

    public WFIBotBuilder setConstraints(double maxVel, double maxAccel, double maxAngVel, double maxAngAccel, double trackWidth){
        this.constraints = new Constraints(maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth);
        return this;
    }

    public RoadRunnerBotEntity build() {
        return new RoadRunnerBotEntity(
                meepMeep,
                constraints,
                width, height,
                startPose, meepMeep.getColorManager().getTheme() , opacity,
                DriveTrainType.MECANUM, false
        );
    }
    public RoadRunnerBotEntity followTrajectorySequence(TrajectorySequence trajectorySequence) {
        RoadRunnerBotEntity bot = this.build();
        bot.followTrajectorySequence(trajectorySequence);
        return bot;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}
