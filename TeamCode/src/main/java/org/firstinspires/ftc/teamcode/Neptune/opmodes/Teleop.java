package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Neptune neptune;

    @Override
    public void initialize() {
        neptune = new Neptune(this);
        // FIGURE OUT HOW TO HANDLE THIS WHEN NOT TESTING
        // THIS WILL NEED TO BE REMOVED, THE POSITION WILL BE SET AFTER AUTO COMPLETES
        //START REMOVE ME
        Pose2d start = new Pose2d(-12, 62, Math.toRadians(90));
        neptune.setStartPosition(start);
        //END REMOVE ME

        schedule(new RunCommand(() -> {
            Pose2d poseEstimate = neptune.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("slide_position", neptune.slides.getSlidePosition());
            telemetry.update();
        }));


        Button liftButton = new GamepadButton(
                neptune.driverOp, GamepadKeys.Button.X
        );

        Button liftButtonDown = new GamepadButton(
                neptune.driverOp, GamepadKeys.Button.Y
        );

        liftButton.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1));


        liftButtonDown.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS));


        Button hangButton = new GamepadButton(
                neptune.driverOp, GamepadKeys.Button.DPAD_UP
        );

        Button hangButtonDown = new GamepadButton(
                neptune.driverOp, GamepadKeys.Button.DPAD_DOWN
        );

        hangButton.whenPressed(new InstantCommand(() -> {
            neptune.hangController.power(0.3);
        })); hangButton.whenReleased(new InstantCommand(() -> {
            neptune.hangController.power(0);
        }));

        hangButtonDown.whenPressed(new InstantCommand(() -> {
            neptune.hangController.power(-0.3);
        })); hangButtonDown.whenReleased(new InstantCommand(() -> {
            neptune.hangController.power(0);
        }));


        MecanumDriveCommand driveCommand = new MecanumDriveCommand(
                neptune.drive, () -> -neptune.driverOp.getLeftY(),
                neptune.driverOp::getLeftX, neptune.driverOp::getRightX
        );

        schedule(driveCommand);
    }

}
