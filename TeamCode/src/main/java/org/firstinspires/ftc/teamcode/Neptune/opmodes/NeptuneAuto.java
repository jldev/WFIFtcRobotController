package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectPawnCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.EndDistanceDriveCommand;
//import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeEjectCommand;
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

import java.util.concurrent.atomic.AtomicReference;

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
        neptune = new Neptune(opMode, Neptune.OpModeType.AUTO, allianceColor);
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
                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1),
                    new WaitCommand(500)
            );
        } else {
            spikeToBackdropCommandGroup = new SequentialCommandGroup(
                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.backdrop)),
                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1),
                    new WaitCommand(500)
            );
        }
        return spikeToBackdropCommandGroup;
    }

    public void run() {

//        opMode.schedule(new RunCommand(() -> {
//            neptune.slides.addTelemetry(opMode.telemetry);
//
//            //neptune.drive.addTelemetry(telemetry);
//            opMode.telemetry.addData("Wall ", neptune.wallDistanceSensor.getDistance(DistanceUnit.INCH));
//            opMode.telemetry.addData("Back ", neptune.distanceSensor.getDistance(DistanceUnit.INCH));
////            opMode.telemetry.addData("Right ", neptune.rightDistanceSensor.getDistance(DistanceUnit.INCH));
////            neptune.vision.addTelemetry(opMode.telemetry);
//            opMode.telemetry.update();
//        }));

        DetectPawnCommand detectPawnCommand = new DetectPawnCommand(neptune.vision);
//        DetectAprilTagCommand detectAprilTagCommand = new DetectAprilTagCommand(neptune.vision, trajectories.targettedAprilTag);

        SequentialCommandGroup depositPixelGroup = new SequentialCommandGroup(
                        new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED),
                        new WaitCommand(1000),
                        new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.CLOSED),
                        new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS)
                );

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
//                                    new IntakeEjectCommand(neptune.intake).withTimeout(NeptuneConstants.NEPTUNE_INTAKE_EJECT_TIME),
                                    new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.NEUTRAL).whenFinished(() -> {
                                        opMode.schedule(getSpikeToBackdropCommandGroup(trajectories).whenFinished(() -> {
                                            opMode.schedule(getBackdropAdjustmentCommand(neptune, trajectories.backdrop, opMode.telemetry).whenFinished(() -> {
                                                opMode.schedule(
                                                        depositPixelGroup.whenFinished(()->{
                                                            if(mStacks) {
                                                                opMode.schedule(new SequentialCommandGroup(
                                                                        new WallLocalizerCommand(neptune, neptune.wallDirection, neptune.wallDistanceSensor, 3),
                                                                        new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 90),
                                                                        new WallLocalizerCommand(neptune, neptune.wallDirection, neptune.wallDistanceSensor, 29),
                                                                        new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.INTAKING),
                                                                        new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 6),
                                                                        new WaitCommand(200),
                                                                        new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 6),
                                                                        new WallLocalizerCommand(neptune, neptune.wallDirection, neptune.wallDistanceSensor, 3),
                                                                        new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 90)
                                                                ).whenFinished(()-> {
                                                                        Pose2d stackDropLocation = trajectories.CenterBackdrop;
                                                                        if(neptune.allianceColor == Neptune.AllianceColor.BLUE && trajectories.backdrop != trajectories.LeftBackdrop){
                                                                            stackDropLocation = trajectories.LeftBackdrop;
                                                                        } else if (neptune.allianceColor == Neptune.AllianceColor.RED && trajectories.backdrop != trajectories.RightBackdrop){
                                                                            stackDropLocation = trajectories.RightBackdrop;
                                                                        }
                                                                        //move to backdrop, deposit pixel, and park
                                                                        opMode.schedule(new SequentialCommandGroup(
                                                                                getBackdropAdjustmentCommand(neptune, stackDropLocation, opMode.telemetry),
                                                                                depositPixelGroup,
                                                                                new WallLocalizerCommand(neptune, neptune.wallDirection, neptune.wallDistanceSensor, 3)));
                                                                    })
                                                                );
                                                            } else { //not mStacks
                                                                    opMode.schedule(new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.park)));
                                                            }
                                                        })
                                                );
                                            }));
                                        }));
                                    })
                            )
                    );
                })
        ));
    }

    private SequentialCommandGroup getBackdropAdjustmentCommand(Neptune neptune, Pose2d backdropLocation, Telemetry telemetry) {
        double wantedDistanceFromWall = 72 - Math.abs(backdropLocation.getY()) - 7;
        return new SequentialCommandGroup(
                new WallLocalizerCommand(neptune, neptune.wallDirection, neptune.wallDistanceSensor, wantedDistanceFromWall),
                new EndDistanceDriveCommand(neptune, MecanumDriveSubsystem.DriveDirection.BACKWARD, neptune.distanceSensor, NeptuneConstants.NEPTUNE_WANTED_DISTANCE_FROM_BACKDROP)
        );
    }
}
