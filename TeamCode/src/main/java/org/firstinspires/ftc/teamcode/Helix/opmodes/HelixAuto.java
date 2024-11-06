package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.CenterOnSpecimenCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Helix.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;

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


    //   TRAJECTORIES

    Pose2d startPos1 = new Pose2d(0, -62, 0);
    Pose2d startPos2 = new Pose2d(0, 0, 0);
    Pose2d startPos3 = new Pose2d(0, 0, 0);

    Pose2d hangPos = new Pose2d(0, 0, 0);

    Pose2d samplePos1 = new Pose2d(0, 0, 0);
    Pose2d samplePos2 = new Pose2d(0, 0, 0);
    Pose2d observationPos  = new Pose2d(0, 0, 0);
    Pose2d wallSpecimenPos = new Pose2d(0, 0, 0);




    public HelixAuto(CommandOpMode commandOpMode, Helix.FieldPos startingPosition, Helix.AllianceColor allianceColor) {
        opMode = commandOpMode;
        helix = new Helix(opMode, Helix.OpModeType.AUTO, allianceColor);
        helix.setStartPosition(startingPosition, allianceColor);
        trajectories = new Trajectories(helix);
        helix.limelight.pipelineSwitch(0);
        helix.limelight.start();
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

//        helix.drive.trajectorySequenceBuilder()



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
