package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import android.drm.DrmStore;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.commands.AutoOutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectPawnCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.OutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SimpleDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Neptune.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;

@Config
@Autonomous(group = "drive", name = "Red Left Auto")
public class RedLeftAuto extends CommandOpMode {

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231127_183907.tflite";
    private static final String[] LABELS = {
            "Pawn",
    };

    //    private MecanumDriveSubsystem drive;
    private Trajectories trajectories;

    private Pose2d start = new Pose2d(-12, 62, Math.toRadians(90));
    private Neptune neptune;

    @Override
    public void initialize() {
        neptune = new Neptune(this);
        neptune.setStartPosition(Neptune.FieldPos.LEFT, Neptune.AllianceColor.RED);
//        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        trajectories = new Trajectories(neptune);

        DetectPawnCommand detectPawnCommand = new DetectPawnCommand(
                new VisionSubsystem(hardwareMap.get(WebcamName.class, "Webcam 1"), TFOD_MODEL_ASSET, LABELS)
        );

        schedule(
                detectPawnCommand.withTimeout(5000).whenFinished(() -> {
                    Trajectories.PropPlacement pawnLocation = detectPawnCommand.getPropLocation(neptune);
                    telemetry.addData("Pawn Location:", pawnLocation);
                    telemetry.update();
                    schedule( new SequentialCommandGroup(
                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getPlacePixelTrajectory(pawnLocation)),
                                    new AutoOutakeStateCommand(neptune.outtake, OutakeSubsystem.AutoOutakeState.OPENED),
                                    new WaitCommand(500),
                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(new Pose2d(55,60))),
                                   new WaitCommand(500),
                                    new SimpleDriveCommand(neptune.drive, MecanumDriveSubsystem.DriveDirection.BACKWARD,10),
//                                   new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(new Pose2d(55,8))),
                                    new WaitCommand(500),
                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getTrajectory(new Pose2d(-24,12))),
                                    new TrajectoryFollowerCommand(neptune.drive, trajectories.getBackdropTrajectory(pawnLocation)),
                                    new WaitCommand(500),
                                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1),
                                    new WaitCommand(1000),
                                    new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED),
                                    new WaitCommand(1000),
                                    new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS)

                            )
                                    .whenFinished(() -> {
                                        telemetry.addLine("trajectory is finished");
                                        telemetry.update();

                                    })
                    );
                })
        );
    }
}
