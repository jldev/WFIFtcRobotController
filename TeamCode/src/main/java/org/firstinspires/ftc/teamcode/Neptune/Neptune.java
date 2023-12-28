package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Neptune.controllers.PIDSlidesController;
import org.firstinspires.ftc.teamcode.Neptune.controllers.SimpleLinearLift;
import org.firstinspires.ftc.teamcode.Neptune.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

public class Neptune {
    public final MecanumDriveSubsystem drive;
    public final SlidesSubsystem slides;
    public final GamepadEx driverOp;
    public final GamepadEx gunnerOp;
    private final MotorEx hangMotor;
    public PIDSlidesController hangController;
    public Pose2d startPos;

    public enum FieldPos {
        LEFT,
        RIGHT
    }

    public FieldPos fieldPos;
    public AllianceColor allianceColor;

    public enum AllianceColor {
        RED,
        BLUE
    }

    public Neptune(CommandOpMode opMode) {

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(opMode.hardwareMap), false);
        slides = new SlidesSubsystem(new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_223),
                new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_223));
        driverOp = new GamepadEx(opMode.gamepad1);
        gunnerOp = new GamepadEx(opMode.gamepad2);
        hangMotor = new MotorEx(opMode.hardwareMap, "leftEncoder", Motor.GoBILDA.RPM_312);
        hangController = new PIDSlidesController(new SimpleLinearLift(hangMotor));

    }

    public void setStartPosition(FieldPos fp, AllianceColor ac) {
        this.fieldPos = fp;
        this.allianceColor = ac;

        if (fp == FieldPos.RIGHT && ac == AllianceColor.RED){
            this.startPos = (new Pose2d(-12, 62, Math.toRadians(90)));

        }
        else if (fp == FieldPos.LEFT && ac == AllianceColor.RED){
            this.startPos = (new Pose2d(42, 62, Math.toRadians(90)));

        } else if (fp == FieldPos.RIGHT && ac == AllianceColor.BLUE){
            this.startPos = (new Pose2d(-12, 62, Math.toRadians(90)));

        } else {
            this.startPos = (new Pose2d(-12, 62, Math.toRadians(90)));

        }
        drive.setPoseEstimate(this.startPos);
    }
}
