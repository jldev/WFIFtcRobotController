package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.firstinspires.ftc.teamcode.Neptune.commands.AutoOutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectAprilTagCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectPawnCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.EndDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeEjectCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeLiftCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.OutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.WallLocalizerCommand;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.function.BooleanSupplier;

public class NeptuneAuto {

    public final Neptune neptune;
    public final Trajectories trajectories;

    private boolean mStacks = false;
    private final CommandOpMode opMode;

    public NeptuneAuto(CommandOpMode commandOpMode, Neptune.FieldPos startingPosition, Neptune.AllianceColor allianceColor, boolean stacks) {
        this(commandOpMode, startingPosition, allianceColor);
        mStacks = stacks;
    }

    public NeptuneAuto(CommandOpMode commandOpMode, Neptune.FieldPos startingPosition, Neptune.AllianceColor allianceColor) {
        opMode = commandOpMode;
        neptune = new Neptune(opMode, Neptune.OpModeType.AUTO);
        neptune.setStartPosition(startingPosition, allianceColor);
        trajectories = new Trajectories(neptune);
    }

    private SequentialCommandGroup getSpikeToBackdropCommandGroup(Trajectories trajectories) {
        SequentialCommandGroup spikeToBackdropCommandGroup;
        if (neptune.fieldPos == Neptune.FieldPos.AU) {
            spikeToBackdropCommandGroup = new SequentialCommandGroup(
                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.stack)),
                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.AUInOut)),
                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.BDInOut)),
                    new WaitCommand(NeptuneConstants.WAIT_FOR_ALLIANCE_PARTNER_TO_CLEAR_MS),
                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.backdrop)),
                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1)
            );
        } else {
            spikeToBackdropCommandGroup = new SequentialCommandGroup(
                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.backdrop)),
                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1)
            );
        }
        return spikeToBackdropCommandGroup;
    }

    public void run() {

        opMode.schedule(new RunCommand(() -> {
            //neptune.drive.addTelemetry(telemetry);
            opMode.telemetry.addData("Back ", neptune.distanceSensor.getDistance(DistanceUnit.INCH));
            opMode.telemetry.addData("Left ", neptune.leftDistanceSensor.getDistance(DistanceUnit.INCH));
            opMode.telemetry.addData("Right ", neptune.rightDistanceSensor.getDistance(DistanceUnit.INCH));
            neptune.vision.addTelemetry(opMode.telemetry);
            opMode.telemetry.update();
        }));

        DetectPawnCommand detectPawnCommand = new DetectPawnCommand(neptune.vision);
        DetectAprilTagCommand detectAprilTagCommand = new DetectAprilTagCommand(neptune.vision, trajectories.targettedAprilTag);

        opMode.schedule(new SequentialCommandGroup(
                new IntakeLiftCommand(neptune.intake, IntakeSubsystem.LiftableIntakePosition.P5),
                detectPawnCommand.withTimeout(3500).whenFinished(() -> {
                    Trajectories.PropPlacement pawnLocation = detectPawnCommand.getPropLocation(neptune);
                    opMode.telemetry.addData("Pawn Location:", pawnLocation);
                    opMode.telemetry.update();

                    // pass in the detected pawn location to the trajectories to set it all up
                    trajectories.setupTrajectories(pawnLocation);
                    opMode.schedule(
                            new SequentialCommandGroup(
                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.spike)),
                                    new IntakeEjectCommand(neptune.intake).withTimeout(NeptuneConstants.NEPTUNE_INTAKE_EJECT_TIME),
                                    new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.NEUTRAL),
                                    getSpikeToBackdropCommandGroup(trajectories),
                                    getBackdropAdjustmentCommand(neptune, trajectories),
                                    new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED),
                                    new WaitCommand(500),
                                    new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.CLOSED),
                                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS)
                            ).whenFinished(() -> {
                                if (neptune.fieldPos == Neptune.FieldPos.BD) {
                                    if (!mStacks) {
                                        opMode.schedule(
                                                new SequentialCommandGroup(
                                                        new WaitCommand(1000),
                                                        new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.park))
                                                ));
                                    } else {

                                        opMode.schedule(new SequentialCommandGroup(
                                                        new WaitCommand(1000),
                                                        new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.BDInOut)),
                                                        new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.AUInOut)),
                                                        new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.stack))

//                                                                                                    new WaitCommand(NeptuneConstants.WAIT_FOR_ALLIANCE_PARTNER_TO_CLEAR_MS),
//                                                                                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.backdrop)),
//                                                                                                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1)
//
                                                )
                                        );
                                    }
                                }
                            })
                    );
                })
        ));
    }

    private Command getBackdropAdjustmentCommand(Neptune neptune, Trajectories trajectories) {
        DistanceSensor ds;
        MecanumDriveSubsystem.DriveDirection direction;
        if (neptune.allianceColor == Neptune.AllianceColor.BLUE) {
            ds = neptune.rightDistanceSensor;
        } else {
            ds = neptune.leftDistanceSensor;
        }

        double wantedDistanceFromWall = 72 - Math.abs(trajectories.backdrop.getY());

        double currentDistance = ds.getDistance(DistanceUnit.INCH);



        if (ds == neptune.rightDistanceSensor)
        {
            if (wantedDistanceFromWall > currentDistance)
            {
                direction = MecanumDriveSubsystem.DriveDirection.LEFT;
            } else
            {
                direction = MecanumDriveSubsystem.DriveDirection.RIGHT;
            }
        } else
        {
            if(wantedDistanceFromWall > currentDistance)
            {
                direction = MecanumDriveSubsystem.DriveDirection.RIGHT;
            } else
            {
                direction = MecanumDriveSubsystem.DriveDirection.LEFT;
            }
        }


        return new SequentialCommandGroup(
                new EndDistanceDriveCommand(neptune, MecanumDriveSubsystem.DriveDirection.BACKWARD, neptune.distanceSensor, NeptuneConstants.NEPTUNE_WANTED_DISTANCE_FROM_BACKDROP),
                // the left right adjustment needs to be here
                new WallLocalizerCommand(neptune, direction, neptune.distanceSensor, wantedDistanceFromWall)
        );
    }
}
