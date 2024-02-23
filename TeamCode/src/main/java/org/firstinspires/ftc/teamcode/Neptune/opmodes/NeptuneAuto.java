package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.firstinspires.ftc.teamcode.Neptune.commands.AutoOutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectAprilTagCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectPawnCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.EndDistanceDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.OutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.function.BooleanSupplier;

public class NeptuneAuto {

    public final Neptune neptune;
    public final Trajectories trajectories;
    private final CommandOpMode opMode;

    public NeptuneAuto(CommandOpMode commandOpMode, Neptune.FieldPos startingPosition, Neptune.AllianceColor allianceColor) {
        opMode = commandOpMode;
        neptune = new Neptune(opMode, Neptune.OpModeType.AUTO);
        neptune.setStartPosition(startingPosition, allianceColor);
        trajectories = new Trajectories(neptune);
    }

    public void run() {

        opMode.schedule(new RunCommand(() -> {
            //neptune.drive.addTelemetry(telemetry);
            opMode.telemetry.addData("distanceSensor", neptune.distanceSensor.getDistance(DistanceUnit.INCH));
            opMode.telemetry.update();
        }));

        DetectPawnCommand detectPawnCommand = new DetectPawnCommand(neptune.vision);

        DetectAprilTagCommand detectAprilTagCommand = new DetectAprilTagCommand(neptune.vision, trajectories.targettedAprilTag);



        opMode.schedule(
                detectPawnCommand.withTimeout(2500).whenFinished(() -> {
                    Trajectories.PropPlacement pawnLocation = detectPawnCommand.getPropLocation(neptune);
                    opMode.telemetry.addData("Pawn Location:", pawnLocation);
                    opMode.telemetry.update();
                    // pass in the detected pawn location to the trajectories to set it all up
                    trajectories.setupTrajectories(pawnLocation);

                    opMode.schedule(new SequentialCommandGroup(
                            new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.spike)),
                            new AutoOutakeStateCommand(neptune.outtake, OutakeSubsystem.AutoOutakeState.OPENED),
                            new WaitCommand(1000),
                            new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.stack)),
//                                    new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.FORWARD, 4),
                            new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.AUInOut)),
                            new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.BDInOut)),
                            new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.backdrop))
                    ).whenFinished(() -> {
                        //Sequential Command Group finished
                        opMode.schedule(detectAprilTagCommand.withTimeout(1500).whenFinished(() -> {
                            CommandBase commandToRun;
                            // when detectAprilTagCommand finished
                            if (detectAprilTagCommand.tagFound){
                                AprilTagPoseFtc ftcPose = detectAprilTagCommand.getPoseFromDetection();
                                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", ftcPose.x, ftcPose.y, ftcPose.z));
                                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", ftcPose.pitch, ftcPose.roll, ftcPose.yaw));
                                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", ftcPose.range, ftcPose.bearing, ftcPose.elevation));

                                commandToRun = new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectoryForAprilTag(detectAprilTagCommand.getPoseFromDetection(), NeptuneConstants.NEPTUNE_WANTED_DISTANCE_FROM_BACKDROP));

                            } else {
                                commandToRun = new EndDistanceDriveCommand(neptune, MecanumDriveSubsystem.DriveDirection.BACKWARD, NeptuneConstants.NEPTUNE_WANTED_DISTANCE_FROM_BACKDROP);

                            }
                            opMode.schedule(commandToRun
                                    .whenFinished(() -> {
                                // when backdrop location finished
                                opMode.schedule(new SequentialCommandGroup(
                                        new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1),
                                        new WaitCommand(2000),
                                        new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED),
                                        new WaitCommand(500),
                                        new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS)
                                ));
                            }));
                        }));
                    }));
                }));
    }
}
