package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Helix.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;

import java.util.ArrayList;

public class HelixAuto {

    public final Helix helix;
    public final Trajectories trajectories;

    private final CommandOpMode opMode;


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

        while(opMode.isStarted()){
            switch (currentState){

                case DRIVETO:
                    new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(desiredPosition));
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
        //opMode.schedule(driveTest(trajectories));
        //opMode.run();
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

    private ArrayList<Task> taskList = new ArrayList<Task>()
    {
        HelixAuto.Task.TRAJECTORY,

    };

}
