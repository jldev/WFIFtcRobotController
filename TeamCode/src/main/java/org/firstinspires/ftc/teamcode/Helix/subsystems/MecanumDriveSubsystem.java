package org.firstinspires.ftc.teamcode.Helix.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class MecanumDriveSubsystem extends SubsystemBase {



    public enum DriveDirection{
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private final SampleMecanumDrive drive;
    private final boolean fieldCentric;

    public MecanumDriveSubsystem(SampleMecanumDrive drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    @Override
    public void periodic(){
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() : 0
        );

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX
                )
        );
    }

    public void driveDirection(DriveDirection direction, double inches){
        switch (direction){

            case FORWARD:
                this.drive.forward(inches);
                break;
            case BACKWARD:
                this.drive.reverse(inches);
                break;
            case LEFT:
                this.drive.left(inches);
                break;
            case RIGHT:
                this.drive.right(inches);
                break;
        }
    }
    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed, boolean slow) {
        if (slow){
            return drive.trajectoryBuilderSlow(startPose, reversed);
        }
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilderSlow(Pose2d startPose){
        return drive.trajectorySequenceBuilderSlow(startPose);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
        return drive.trajectorySequenceBuilderSlow(startPose);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void followTrajectorySequence(TrajectorySequence sequence){
        drive.followTrajectorySequenceAsync(sequence);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void waitForIdle(){
        drive.waitForIdle();
    }
    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public void addTelemetry(Telemetry telemetry){
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }
}