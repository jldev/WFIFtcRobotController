package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.OutakeStateCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.DroneLauncherSubsytem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Neptune neptune;

    @Override
    public void initialize() {
        neptune = new Neptune(this, Neptune.OpModeType.TELEOP);

        schedule(new RunCommand(() -> {
            //neptune.drive.addTelemetry(telemetry);
            neptune.slides.addTelemetry(telemetry);
            telemetry.update();
        }));

        // Slide System Buttons
        neptune.liftButton.whenPressed(new InstantCommand(() -> neptune.slides.changeToNextSlidePosition()));
        neptune.liftButtonDown.whenPressed(new SlidePositionCommand(neptune.slides, SlidesSubsystem.SlidesPosition.HOME_POS));

        // Manual Slides Button
        neptune.manualSlideButtonUp.whileHeld(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.UP);}));
        neptune.manualSlideButtonUp.whenReleased(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.OFF);}));
        neptune.manualSlideButtonDown.whileHeld(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.DOWN);}));
        neptune.manualSlideButtonUp.whenReleased(new InstantCommand(() -> {neptune.slides.manualSlideControl(SlidesSubsystem.ManualControlDirection.OFF);}));


        //Drone Launcher with a safety switch
        neptune.droneLauncherButton.and(neptune.droneLauncherButton2).whenActive(new InstantCommand(() -> {neptune.launcher.setDroneState(DroneLauncherSubsytem.droneLaunchState.LAUNCH);}));

        // Hang System Buttons
        neptune.hangButtonUp.whileHeld(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.UP);}));
        neptune.hangButtonUp.whenReleased(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.STOPPED);}));


        // Hang System Buttons
        neptune.hangButtonDown.whileHeld(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.DOWN);}));
        neptune.hangButtonDown.whenReleased(new InstantCommand(() -> {neptune.hang.hangDirection(HangSubsystem.HangMotorDirection.STOPPED);}));


        // Hang Servo Buttons
        neptune.hangArmButtonUp.whenPressed(new InstantCommand(() -> {neptune.hang.changeState();}));


        // Cowbell Outtake Button
        neptune.outtakeButton.whileHeld(new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.OPENED));
        neptune.outtakeButton.whenReleased(new OutakeStateCommand(neptune.outtake, OutakeSubsystem.OutakeState.CLOSED));

        // Intake Buttons
        neptune.intakeButton.whileHeld(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.INTAKING));
        neptune.intakeButton.whenReleased(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.NEUTRAL));

        neptune.intakeReverseButton.whileHeld(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.EJECTING));
        neptune.intakeReverseButton.whenReleased(new IntakeStateCommand(neptune.intake, IntakeSubsystem.IntakeState.NEUTRAL));

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(

//                neptune.drive, () -> -neptune.driverOp.getLeftY(),
//                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,

                neptune.drive, () -> neptune.driverOp.getRightY(),
                () -> neptune.driverOp.getRightX(), () -> neptune.driverOp.getLeftX(),
                neptune.driveBrakeTrigger::get
        );
        neptune.drive.setDefaultCommand(driveCommand);

    }

}
