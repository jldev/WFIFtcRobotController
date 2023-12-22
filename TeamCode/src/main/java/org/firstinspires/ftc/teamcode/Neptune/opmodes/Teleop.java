package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Neptune.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.SlidePositionCommand;
import org.firstinspires.ftc.teamcode.Neptune.controllers.PIDSlidesController;
import org.firstinspires.ftc.teamcode.Neptune.controllers.SimpleLinearLift;
import org.firstinspires.ftc.teamcode.Neptune.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

import java.io.CharArrayWriter;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveCommand driveCommand;
    private SlidesSubsystem slides;
    private GamepadEx driverOp;
//    private PIDSlidesController slideController;
    private PIDSlidesController hangController;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        slides = new SlidesSubsystem(new MotorEx(hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312),
                new MotorEx(hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312));
        driverOp = new GamepadEx(gamepad1);
//        MotorEx liftMotor = new MotorEx(hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312);
        MotorEx hangMotor = new MotorEx(hardwareMap, "leftEncoder", Motor.GoBILDA.RPM_312);
        // We may need to tweak this value hangMotor.setPositionCoefficient();
//        liftMotor.setInverted(false);
//        liftMotor.encoder.setDirection(Motor.Direction.REVERSE);
        // We may need to tweak this value liftMotor.setPositionCoefficient();

//        slideController = new PIDSlidesController(new SimpleLinearLift(liftMotor));
//        liftMotor.setPositionTolerance(100);
        hangController = new PIDSlidesController(new SimpleLinearLift(hangMotor));
//        slideController.resetStage();
        // FIGURE OUT HOW TO HANDLE THIS WHEN NOT TESTING
        Pose2d start = new Pose2d(-12, 62, Math.toRadians(90));
        drive.setPoseEstimate(start);

        schedule(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("liftpos", slideController.getPosition());
            telemetry.update();
        }));

        Button liftButton = new GamepadButton(
                driverOp, GamepadKeys.Button.X
        );

        Button liftButtonDown = new GamepadButton(
                driverOp, GamepadKeys.Button.Y
        );

        liftButton.whenPressed(new SlidePositionCommand(slides, SlidesSubsystem.SlidesPosition.POSITION_1));


        liftButtonDown.whenPressed(new SlidePositionCommand(slides, SlidesSubsystem.SlidesPosition.HOME_POS));


        Button hangButton = new GamepadButton(
                driverOp, GamepadKeys.Button.DPAD_UP
        );

        Button hangButtonDown = new GamepadButton(
                driverOp, GamepadKeys.Button.DPAD_DOWN
        );

        hangButton.whenPressed(new InstantCommand(() -> {
            hangController.power(0.3);
        })); hangButton.whenReleased(new InstantCommand(() -> {
            hangController.power(0);
        }));

        hangButtonDown.whenPressed(new InstantCommand(() -> {
            hangController.power(-0.3);
        })); hangButtonDown.whenReleased(new InstantCommand(() -> {
            hangController.power(0);
        }));



        driveCommand = new MecanumDriveCommand(
                drive, () -> -driverOp.getLeftY(),
                driverOp::getLeftX, driverOp::getRightX
        );

        schedule(driveCommand);
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
//        slideController.pidUpdate();
    }

}
