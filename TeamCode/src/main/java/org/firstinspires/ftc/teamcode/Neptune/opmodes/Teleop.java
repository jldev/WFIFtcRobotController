package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.firstinspires.ftc.teamcode.Neptune.commands.AutoOutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeLiftCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.OutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

import java.util.logging.Level;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Neptune neptune;

    @Override
    public void initialize() {
        neptune = new Neptune(this);

        schedule(new RunCommand(() -> {
//            Pose2d poseEstimate = neptune.drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("slide_position", neptune.slides.getSlidePosition());
//            telemetry.update();
            neptune.slides.addTelemetry(telemetry);
            telemetry.update();
        }));

        neptune.liftButton.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1));
        neptune.liftButtonDown.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS));

        neptune.hangButton.whenPressed(new InstantCommand(() -> {neptune.hangController.power(NeptuneConstants.NEPTUNE_HANG_MOTOR_POWER);}));
        neptune.hangButton.whenReleased(new InstantCommand(() -> {neptune.hangController.power(0);}));

        neptune.hangButtonDown.whenPressed(new InstantCommand(() -> {neptune.hangController.power(-NeptuneConstants.NEPTUNE_HANG_MOTOR_POWER);}));
        neptune.hangButtonDown.whenReleased(new InstantCommand(() -> {neptune.hangController.power(0);}));

        neptune.outtakeButton.whileHeld(new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED));
        neptune.outtakeButton.whenReleased(new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.CLOSED));
        neptune.outtakeButton.whileHeld(new AutoOutakeStateCommand(neptune.outtake, OutakeSubsystem.AutoOutakeState.OPENED));
        neptune.outtakeButton.whenReleased(new AutoOutakeStateCommand(neptune.outtake, OutakeSubsystem.AutoOutakeState.CLOSED));

        neptune.intakeliftbutton.whileHeld(new IntakeLiftCommand(neptune.intake, IntakeSubsystem.LiftableIntakePosition.RAISE));
        neptune.intakeliftbutton.whenReleased(new IntakeLiftCommand(neptune.intake, IntakeSubsystem.LiftableIntakePosition.LOWER));

        neptune.intakeButton.toggleWhenPressed(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.INTAKING));
        neptune.intakeReverseButton.toggleWhenPressed(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.EJECTING));
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(
                neptune.drive, () -> -neptune.driverOp.getLeftY(),
                neptune.driverOp::getLeftX, neptune.driverOp::getRightX
        );
        neptune.drive.setDefaultCommand(driveCommand);

    }

}
