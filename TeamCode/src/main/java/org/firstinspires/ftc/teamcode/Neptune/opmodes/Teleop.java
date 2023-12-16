package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neptune.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveCommand driveCommand;
    private GamepadEx driverOp;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);
        driverOp = new GamepadEx(gamepad1);

        // FIGURE OUT HOW TO HANDLE THIS WHEN NOT TESTING
        Pose2d start = new Pose2d(-12, 62, Math.toRadians(90));
        drive.setPoseEstimate(start);

        schedule(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }));

        driveCommand = new MecanumDriveCommand(
                drive, () -> -driverOp.getLeftY(),
                driverOp::getLeftX, driverOp::getRightX
        );

        schedule(driveCommand);
    }
}
