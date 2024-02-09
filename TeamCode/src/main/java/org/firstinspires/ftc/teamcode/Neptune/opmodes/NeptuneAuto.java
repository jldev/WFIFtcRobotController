package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.commands.AutoOutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectPawnCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.OutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;

public class NeptuneAuto {

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231127_183907.tflite";
    private static final String[] LABELS = {
            "Pawn",
    };

    private final Neptune neptune;
    private final Trajectories trajectories;
    private final CommandOpMode opMode;

    public NeptuneAuto(CommandOpMode commandOpMode, Neptune.FieldPos startingPosition, Neptune.AllianceColor allianceColor) {
        opMode = commandOpMode;
        neptune = new Neptune(opMode);
        neptune.setStartPosition(startingPosition, allianceColor);
        trajectories = new Trajectories(neptune);
    }

    public void run() {

        DetectPawnCommand detectPawnCommand = new DetectPawnCommand(
                new VisionSubsystem(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), TFOD_MODEL_ASSET, LABELS)
        );

        SequentialCommandGroup pixelDeliveryGroup = new SequentialCommandGroup(new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.backdrop)),
                new WaitCommand(500),
                new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1),
                new WaitCommand(1000),
                new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED),
                new WaitCommand(1000),
                new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS));

        SequentialCommandGroup audienceEntryGroup = new SequentialCommandGroup(new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.BDInOut)),
                new TrajectoryFollowerCommand(neptune.drive,trajectories.getTrajectory(trajectories.AUInOut)));

        SequentialCommandGroup backdropEntryGroup = new SequentialCommandGroup(new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.AUInOut)),
                new TrajectoryFollowerCommand(neptune.drive,trajectories.getTrajectory(trajectories.BDInOut)));

        opMode.schedule(
                detectPawnCommand.withTimeout(5000).whenFinished(() -> {
                    Trajectories.PropPlacement pawnLocation = detectPawnCommand.getPropLocation(neptune);
                    opMode.telemetry.addData("Pawn Location:", pawnLocation);
                    opMode.telemetry.update();
                    // pass in the detected pawn location to the trajectories to set it all up
                    trajectories.setupTrajectories(pawnLocation);

                    opMode.schedule(new SequentialCommandGroup(
                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(trajectories.spike)),
                                    new AutoOutakeStateCommand(neptune.outtake, OutakeSubsystem.AutoOutakeState.OPENED),
                                    new WaitCommand(500)
                                    ).whenFinished((() -> {
                                      if (neptune.fieldPos == Neptune.FieldPos.AU){

                                      }

                                      opMode.schedule(pixelDeliveryGroup);
                            })
//                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(new Pose2d(55, 60))),
//                                    new WaitCommand(500),
//                                    new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD, 10),
//                                   new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(new Pose2d(55,8))),
//                                    new WaitCommand(500),
//                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(new Pose2d(-24, 12))),


                            ).whenFinished(() -> {
                                opMode.telemetry.addLine("trajectory is finished");
                                opMode.telemetry.update();
                            })
                    );
                })
        );
    }
}
