package org.firstinspires.ftc.teamcode.Helix.opmodes;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.HelixConstants;
import org.firstinspires.ftc.teamcode.Helix.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.SlideSubsystem;


//values
import static org.firstinspires.ftc.teamcode.Helix.subsystems.IntakeSubsystem.GripperState.OPEN;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Helix helix;

    @Override
    public void initialize() {
        helix = new Helix(this, Helix.OpModeType.TELEOP, Helix.AllianceColor.RED);

        this.schedule(new RunCommand(() -> {
            helix.slides.addTelemetry(telemetry);
            telemetry.update();
        }));

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(

//                neptune.drive, () -> -neptune.driverOp.getLeftY(),
//                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,

                helix.drive, () -> helix.driverOp.getRightY(),
                () -> helix.driverOp.getRightX(), () -> helix.driverOp.getLeftX()
        );
        helix.drive.setDefaultCommand(driveCommand);




        //         INTAKE

        helix.instakeGripperButton.whileHeld(helix.intake.setGripperOpen());
        helix.instakeGripperButton.whenReleased(helix.intake.setGripperClosed());

        helix.intakeLiftButton.toggleWhenPressed(new InstantCommand(() -> helix.intake.cycleLift()));



        //         SLIDES

        // Manual Slides Button
        helix.tempSlideUpButton.whileHeld(new InstantCommand(() -> {helix.slides.manualSlideControl(SlideSubsystem.ManualControlDirection.UP);}));
        helix.tempSlideUpButton.whenReleased(new InstantCommand(() -> {helix.slides.manualSlideControl(SlideSubsystem.ManualControlDirection.OFF);}));
        helix.tempSlideDownButton.whileHeld(new InstantCommand(() -> {helix.slides.manualSlideControl(SlideSubsystem.ManualControlDirection.DOWN);}));
        helix.tempSlideDownButton.whenReleased(new InstantCommand(() -> {helix.slides.manualSlideControl(SlideSubsystem.ManualControlDirection.OFF);}));

        // Slide Presets
        helix.home_slidePreset.whenPressed(new InstantCommand(() -> {helix.slides.changeToSlidePosition(SlideSubsystem.SlidePosition.HOME);}));
        helix.wall_slidePreset.whenPressed(new InstantCommand(() -> {helix.slides.changeToSlidePosition(SlideSubsystem.SlidePosition.WALL);}));
        helix.hang_slidePreset.whenPressed(new InstantCommand(() -> {helix.slides.changeToSlidePosition(SlideSubsystem.SlidePosition.HANG);}));
        helix.basket_slidePreset.whenPressed(new InstantCommand(() -> {helix.slides.changeToSlidePosition(SlideSubsystem.SlidePosition.BASKET);}));



        //    HANG
        helix.hangRaise.whileHeld(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.UP);}));
        helix.hangRaise.whenReleased(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.OFF);}));
        helix.hangLower.whileHeld(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.DOWN);}));
        helix.hangLower.whenReleased(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.OFF);}));

    }

}
