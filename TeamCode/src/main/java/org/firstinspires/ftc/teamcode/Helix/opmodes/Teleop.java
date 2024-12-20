package org.firstinspires.ftc.teamcode.Helix.opmodes;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.SlideSubsystem;


//values


@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Helix helix;

    @Override
    public void initialize() {
        helix = new Helix(this, Helix.OpModeType.TELEOP, Helix.AllianceColor.RED);

//        this.schedule(new RunCommand(() -> {
//            helix.slides.addTelemetry(telemetry);
//            helix.pivot.addTelemetry(telemetry);
//            telemetry.update();
//        }));

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(

//                neptune.drive, () -> -neptune.driverOp.getLeftY(),
//                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,

                helix.drive, () -> helix.driverOp.getRightY(),
                () -> helix.driverOp.getRightX(), () -> helix.driverOp.getLeftX()
        );
        helix.drive.setDefaultCommand(driveCommand);






        //         SLIDES

        // Manual Slides Button
        helix.verticleSlideUp.whileHeld(new InstantCommand(() -> {helix.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.UP);}));
        helix.verticleSlideUp.whenReleased(new InstantCommand(() -> {helix.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.OFF);}));
        helix.verticleSlideDown.whileHeld(new InstantCommand(() -> {helix.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.DOWN);}));
        helix.verticleSlideDown.whenReleased(new InstantCommand(() -> {helix.slides.verticalManualSlideControl(SlideSubsystem.VerticalManualControlDirection.OFF);}));

        helix.horizontalSlideOut.whileHeld(new InstantCommand(() -> {helix.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.OUT);}));
        helix.horizontalSlideOut.whenReleased(new InstantCommand(() -> {helix.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.OFF);}));
        helix.horizontalSlideIn.whileHeld(new InstantCommand(() -> {helix.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.IN);}));
        helix.horizontalSlideIn.whenReleased(new InstantCommand(() -> {helix.slides.horizontalManualSlideControl(SlideSubsystem.HorizontalManualControlDirection.OFF);}));



        //  Presets
        helix.home_slidePreset.whenPressed(helix.GoSub());

        helix.wall_slidePreset.whenPressed(helix.GoWall());

        helix.hang_slidePreset.whenPressed(helix.GoHang());

        helix.basket_slidePreset.whenPressed(helix.GoBasket());


        //    HANG
//        helix.hangRaise.whileHeld(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.UP);}));
//        helix.hangRaise.whenReleased(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.OFF);}));
//        helix.hangLower.whileHeld(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.DOWN);}));
//        helix.hangLower.whenReleased(new InstantCommand(() -> {helix.hang.manualSlideControl(HangSubsystem.ManualControlDirection.OFF);}));


        //   PIVOT
        helix.pivotRaise.whileHeld(new InstantCommand(() -> {helix.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.UP);}));
        helix.pivotRaise.whenReleased(new InstantCommand(() -> {helix.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.OFF);}));
        helix.pivotLower.whileHeld(new InstantCommand(() -> {helix.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.DOWN);}));
        helix.pivotLower.whenReleased(new InstantCommand(() -> {helix.pivot.ManualPivotControl(PivotSubsystem.ManualControlDirection.OFF);}));

        // Pivot Presets
        helix.home_pivotPreset.whenPressed(new InstantCommand(() -> {helix.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.HOME);}));
        helix.hang_pivotPreset.whenPressed(new InstantCommand(() -> {helix.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.HANG);}));
        helix.basket_pivotPreset.whenPressed(new InstantCommand(() -> {helix.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.BASKET);}));
        helix.sub_pivotPreset.whenPressed(new InstantCommand(() -> {helix.pivot.changeToSlidePosition(PivotSubsystem.SlidePosition.SUB);}));
    }

}
