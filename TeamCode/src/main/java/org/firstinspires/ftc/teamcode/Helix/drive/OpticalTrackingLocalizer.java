package org.firstinspires.ftc.teamcode.Helix.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

public class OpticalTrackingLocalizer implements Localizer {
    Pose2d mPoseEstimate;
    Pose2d mPoseVelocity;

    Pose2d mPreviousPose;

    double previousTime;
    public OpticalTrackingLocalizer(){
        mPoseEstimate = new Pose2d(0,0, 0);
        mPoseVelocity = new Pose2d(0,0, 0);
    }
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        mPoseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return mPoseVelocity;
    }

    @Override
    public void update() {
        // set pose estimate here from optical sensor mPoseEstimate

        // set pose velocity here from optical sensor

        double runtime = 0;
        double currentTime = 0;

        Pose2d mDistance = new Pose2d(
                mPoseEstimate.getX() - mPreviousPose.getX(),
                mPoseEstimate.getY() - mPreviousPose.getY(),
                mPoseEstimate.getHeading() - mPreviousPose.getHeading());

        double time = runtime - previousTime;

        //    im pretty sure the above either works or works backwards




        mPoseVelocity = new Pose2d(
                mDistance.getX() / time,
                mDistance.getY() / time,
                mDistance.getHeading() / time);


        mPreviousPose = mPoseEstimate;
        previousTime = currentTime;

    }
}
