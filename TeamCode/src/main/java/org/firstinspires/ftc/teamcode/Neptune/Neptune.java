package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.controllers.PIDSlidesController;
import org.firstinspires.ftc.teamcode.Neptune.controllers.SimpleLinearLift;
import org.firstinspires.ftc.teamcode.Neptune.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.PIDMotor;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;

import java.util.function.BooleanSupplier;

public class Neptune {
    public final MecanumDriveSubsystem drive;
    public final OutakeSubsystem outtake;
    public final IntakeSubsystem intake;
    public final SlidesSubsystem slides;
    public final GamepadEx driverOp;
    public final GamepadEx gunnerOp;
    private final MotorEx hangMotor;
    public final GamepadButton liftButton;
    public final GamepadButton liftButtonDown;
    public final GamepadButton outtakeButton;
    public final GamepadButton intakeliftbutton;
    public final GamepadButton hangButton;
    public final GamepadButton hangButtonDown;
    public final GamepadButton intakeReverseButton;
    private final Trigger leftTrigger;
    private final Trigger rightTrigger;
    public PIDSlidesController hangController;
    public Pose2d startPos;

    public enum FieldPos {
        AU,
        BD
    }

    public FieldPos fieldPos;
    public AllianceColor allianceColor;

    public enum AllianceColor {
        RED,
        BLUE
    }

    public Neptune(CommandOpMode opMode) {

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(opMode.hardwareMap), false);
        outtake = new OutakeSubsystem(opMode.hardwareMap.get(Servo.class, "outtakeServo"), opMode.hardwareMap.get(Servo.class, "autoOuttake"));
        intake = new IntakeSubsystem(new MotorEx(opMode.hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_1620),
                opMode.hardwareMap.get(Servo.class, "intakeServo1"), opMode.hardwareMap.get(Servo.class, "intakeServo2"));
        slides = new SlidesSubsystem(new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312),
                new PIDMotor(opMode.hardwareMap, "vbarMotor", 537.6, 340));
        driverOp = new GamepadEx(opMode.gamepad1);
        gunnerOp = new GamepadEx(opMode.gamepad2);
        hangMotor = new MotorEx(opMode.hardwareMap, "hangMotor", Motor.GoBILDA.RPM_312);
        hangController = new PIDSlidesController(new SimpleLinearLift(hangMotor));

        // register subsystems
        opMode.register(drive);
        opMode.register(outtake);
        opMode.register(intake);
        opMode.register(slides);

        // driver button setup
        liftButton = new GamepadButton(gunnerOp, GamepadKeys.Button.X);
        liftButtonDown = new GamepadButton(gunnerOp, GamepadKeys.Button.Y);
        outtakeButton = new GamepadButton(gunnerOp, GamepadKeys.Button.B);
        intakeReverseButton = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_LEFT);
        intakeliftbutton = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);
        hangButton = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP);
        hangButtonDown = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN);


        // gunner button setup
        TriggerReader reader = new TriggerReader(gunnerOp, GamepadKeys.Trigger.LEFT_TRIGGER, 100);

        BooleanSupplier triggerDown = () -> reader.isDown();
        BooleanSupplier triggerUp = () -> !reader.isDown();

        leftTrigger = new Trigger(triggerDown);
        rightTrigger = new Trigger(triggerUp);


    }



    public void setStartPosition(FieldPos fp, AllianceColor ac) {
        this.fieldPos = fp;
        this.allianceColor = ac;

        if (fp == FieldPos.BD){
            this.startPos = (new Pose2d(-12, 62, Math.toRadians(90)));

        }
        else if (fp == FieldPos.AU){
            this.startPos = (new Pose2d(42, 62, Math.toRadians(90)));

        }
        if(ac == AllianceColor.BLUE){
            this.startPos = Trajectories.translatePosePositionToBlue(this.startPos);
        }

        drive.setPoseEstimate(this.startPos);
    }
}
