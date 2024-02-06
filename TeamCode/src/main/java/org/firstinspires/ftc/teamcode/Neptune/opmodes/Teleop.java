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
import org.firstinspires.ftc.teamcode.Neptune.subsystems.HangSubsystem;
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
            //neptune.drive.addTelemetry(telemetry);
            neptune.slides.addTelemetry(telemetry);
            telemetry.update();
        }));

        // Lift System Buttons
        neptune.liftButton.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.POSITION_1));
        neptune.liftButtonDown.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS));

        neptune.manualSlideButtonUp.whileHeld(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.UP);}));
        neptune.manualSlideButtonUp.whenReleased(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.OFF);}));
        neptune.manualSlideButtonDown.whileHeld(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.DOWN);}));
        neptune.manualSlideButtonUp.whenReleased(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.OFF);}));
        //neptune.slideOffsetIncrease.whileHeld(new InstantCommand(() -> {neptune.slides.UpdateOffset(1);}));
        //neptune.slideOffsetDecrease.whileHeld(new InstantCommand(() -> {neptune.slides.UpdateOffset(-1);}));

        // Hang System Buttons
        neptune.hangButtonUp.whileHeld(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.UP);}));
        neptune.hangButtonUp.whenReleased(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.STOPPED);}));

        neptune.hangButtonDown.whileHeld(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.DOWN);}));
        neptune.hangButtonDown.whenReleased(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.STOPPED);}));

        neptune.hangArmButtonUp.whenActive(new InstantCommand(() -> {neptune.hang.setHangState(HangSubsystem.HangState.HANGING);}));
        neptune.hangArmButtonDown.whenActive(new InstantCommand(() -> {neptune.hang.setHangState(HangSubsystem.HangState.REST);}));

        // Cowbell Outtake Button
        neptune.outtakeButton.whileHeld(new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED));
        neptune.outtakeButton.whenReleased(new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.CLOSED));

        // Intake Buttons
        neptune.intakeliftbutton.whileHeld(new IntakeLiftCommand(neptune.intake, IntakeSubsystem.LiftableIntakePosition.LOWER));
        neptune.intakeliftbutton.whenReleased(new IntakeLiftCommand(neptune.intake, IntakeSubsystem.LiftableIntakePosition.RAISE));

        neptune.intakeReverseButton.toggleWhenPressed(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.EJECTING));

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(
                neptune.drive, () -> -neptune.driverOp.getLeftY(),
                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,
                neptune.driveBrakeTrigger::get
        );
        neptune.drive.setDefaultCommand(driveCommand);

    }

}
