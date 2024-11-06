package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.CenterOnSpecimenCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.Helix.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

public class HelixAuto {

    public Helix helix;
    public Trajectories trajectories;

    private CommandOpMode opMode;


    public Pose2d desiredPosition;

    private enum Task{
        DRIVETO,
        RETRIEVE_SPECIMEN,
        DEPOSIT_SPECIMEN,
        RETRIEVE_SAMPLE,
        DEPOSIT_SAMPLE,
    }

    public HelixAuto(CommandOpMode commandOpMode, Helix.FieldPos startingPosition, Helix.AllianceColor allianceColor) {
        opMode = commandOpMode;
        helix = new Helix(opMode, Helix.OpModeType.AUTO, allianceColor);
        helix.setStartPosition(startingPosition, allianceColor);
        trajectories = new Trajectories(helix);
        //helix.limelight.pipelineSwitch(0);
        //helix.limelight.start();
        opMode.schedule(new CenterOnSpecimenCommand(helix));
    }

    private SequentialCommandGroup driveTest(Trajectories trajectories) {
        return new SequentialCommandGroup(
                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 12)
//                new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(trajectories.waypoint1)),
//                new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(trajectories.waypoint2))


//   Get basic auto working -- put Pose2ds in array and test PathTo
                );
    }




    public void run() {
        Task currentState = Task.DRIVETO;

        Pose2d startPos = new Pose2d(14, -62, Math.toRadians(90));
        helix.setStartPosition(startPos);
        TrajectorySequence initialTrajectory = helix.drive.trajectorySequenceBuilder(startPos)
                .lineToConstantHeading(new Vector2d(10, -36))
                .lineToConstantHeading(new Vector2d(34, -36))
                .lineToConstantHeading(new Vector2d(34, -9))
                .lineToConstantHeading(new Vector2d(44, -9))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(44, -50))
                .lineToConstantHeading(new Vector2d(44, -9))
                .lineToConstantHeading(new Vector2d(56, -9))
                .lineToConstantHeading(new Vector2d(56, -50))
                .build();

        opMode.schedule(new TrajectorySequenceFollowerCommand(helix.drive, initialTrajectory));

//                if(xAngle < -8f)
//
//                {
//                    helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.LEFT, 1.5);
//                    opMode.telemetry.addData("XAngle within big threshold", xAngle);
//                } else if (xAngle < -2.5) {
//                    helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.LEFT, .35);
//                    opMode.telemetry.addData("XAngle within small threshold", xAngle);
//                } else
//                {
//                    opMode.telemetry.addLine("Destination reached");
//                }
//
//
//                if(xAngle > 8f)
//                {
//                    helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.RIGHT, 1.5);
//                    opMode.telemetry.addData("XAngle within big threshold", xAngle);
//                } else if (xAngle > 2.5) {
//                    helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.RIGHT, .35);
//                    opMode.telemetry.addData("XAngle within small threshold", xAngle);
//                } else
//                {
//                    opMode.telemetry.addLine("Destination reached");
//                }
//                opMode.telemetry.update();

                //make center -- like get it working then try to make a class idfk im screwed.


        switch (currentState) {

            case DRIVETO:
//                    new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(desiredPosition));
                break;
            case RETRIEVE_SPECIMEN:
                break;
            case DEPOSIT_SPECIMEN:
                break;
            case RETRIEVE_SAMPLE:
                break;
            case DEPOSIT_SAMPLE:
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }
    }






    private void PathTo(Pose2d position)
    {
        new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(position));
    }


    private void PathToArray(Pose2d[] positions)
    {
        for(Pose2d pos : positions)
        {
            PathTo(pos);
        }
    }

//    private ArrayList<Task> taskList = new ArrayList<Task>()
//    {
//        HelixAuto.Task.TRAJECTORY,
//
//    };

}
