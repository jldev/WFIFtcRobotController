package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Helix.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Helix.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.SlideSubsystem;

public class HelixAuto {

    public Helix helix;
    public Trajectories trajectories;

    private CommandOpMode opMode;
    Task currentState = Task.PRELOAD_DRIVE;

    public Pose2d desiredPosition;

    private enum Task{
        PRELOAD_DRIVE,
        RETRIEVE_SPECIMEN,
        DEPOSIT_SPECIMEN,
        RETRIEVE_SAMPLE,
        DEPOSIT_SAMPLE,
        WAIT_FOR_DRIVE,
        PUSH_SAMPLES,
        PARK,
        PARK_BASKET
    }

    private int runCount = 0;

    public HelixAuto(CommandOpMode commandOpMode, Helix.FieldPos startingPosition, Helix.AllianceColor allianceColor, Helix.Target target) {
        opMode = commandOpMode;
        helix = new Helix(opMode, Helix.OpModeType.AUTO, allianceColor);
        helix.setStartPosition(startingPosition, allianceColor);
        helix.target = target;
        if(helix.target == Helix.Target.SPECIMENS)
        {
            currentState = Task.PRELOAD_DRIVE;
        } else
        {
            currentState = Task.DEPOSIT_SAMPLE;
        }
        Pose2d startPos = new Pose2d(12, -60, Math.toRadians(270));
        helix.setStartPosition(startPos);

        trajectories = new Trajectories(helix);
        //helix.limelight.pipelineSwitch(0);
        //helix.limelight.start();
//        opMode.schedule(new CenterOnSpecimenCommand(helix));
    }




    public void run() {
        opMode.telemetry.addData("Current State", currentState);
        opMode.telemetry.addData("Run Count", runCount++);
        opMode.telemetry.update();
        switch (currentState) {

            case PRELOAD_DRIVE:
                opMode.schedule(
                        new SequentialCommandGroup(
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 15),
                                helix.GoHang(),
                                new WaitCommand(1000),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 15).whenFinished(()->{
                                    currentState = Task.DEPOSIT_SPECIMEN;
                                })
                            )
                        );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case WAIT_FOR_DRIVE:
                break;
            case RETRIEVE_SPECIMEN:
                break;
            case DEPOSIT_SPECIMEN:
                helix.claw.SetClawGripState(ClawSubsystem.GripState.OPEN);
                if(helix.pushSamples)
                {
                    currentState = Task.PUSH_SAMPLES;
                } else
                {
                    currentState = Task.PARK;
                }
                break;
            case RETRIEVE_SAMPLE:
                break;
            case DEPOSIT_SAMPLE:
                opMode.schedule(
                        new SequentialCommandGroup(
                                helix.GoPreloadBasket(),
                                new WaitCommand(5000),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 5)
                                        .whenFinished(() -> currentState = Task.PARK_BASKET)
                        )

                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case PUSH_SAMPLES:
                opMode.schedule(
                        new SequentialCommandGroup(
                                helix.GoWall(),
                                new WaitCommand(2000),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 10),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 31),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 41),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 12),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 47)
                        )
                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case PARK:
                opMode.schedule(
                        new SequentialCommandGroup(
                                helix.GoWall(),
                                new WaitCommand(2000),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 18),
                                new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 48),
                                new InstantCommand(() -> {helix.claw.SetClawGripState(ClawSubsystem.GripState.OPEN);}),
                                new WaitCommand(500).whenFinished(() -> currentState = Task.PARK_BASKET)
                        )
                );
                currentState = Task.WAIT_FOR_DRIVE;
                break;
            case PARK_BASKET:
                opMode.schedule(
                        new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 5),
                        helix.GoSub(),
                        new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.LEFT, 50),
                        new SimpleDriveCommand(helix.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 8)
                );
                currentState = Task.WAIT_FOR_DRIVE;
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
