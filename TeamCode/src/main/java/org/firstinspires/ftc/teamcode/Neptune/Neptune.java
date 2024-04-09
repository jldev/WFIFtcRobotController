package org.firstinspires.ftc.teamcode.Neptune;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.DroneLauncherSubsytem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.IndicatorSubsytem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.VisionSubsystem;

public class Neptune {

    public VisionSubsystem vision;
    public final MecanumDriveSubsystem drive;
    public final OutakeSubsystem outtake;
    public final IntakeSubsystem intake;
    public final SlidesSubsystem slides;

    public final DroneLauncherSubsytem launcher;

    public  final IndicatorSubsytem indicatorSubsytem;

    public final HangSubsystem hang;
    public final GamepadEx driverOp;
    public final GamepadEx gunnerOp;
    public final Servo hangServo;
    public final Servo hangServo2;
    public final MotorEx hangMotor;
    public final GamepadButton liftButton;
    public final GamepadButton liftButtonDown;
    public final GamepadButton outtakeButton;
    public final GamepadButton intakeButton;
    public final GamepadButton hangButtonUp;
    public final GamepadButton hangButtonDown;
    public final GamepadButton hangArmButtonUp;
    public final GamepadButton intakeReverseButton;

    public final GamepadButton droneLauncherButton;

    public final GamepadButton droneLauncherButton2;

    public final SwitchReader magSwitchButton;

    public final DistanceSensor distanceSensor;
    public final GamepadTriggerAsButton manualSlideButtonUp;
    public final GamepadTriggerAsButton manualSlideButtonDown;
    public final GamepadTrigger driveBrakeTrigger;
    public Pose2d startPos;

    public enum FieldPos {
        AU,
        BD
    }

    public FieldPos fieldPos;
    public AllianceColor allianceColor;

    public final CommandOpMode mOpMode;
    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum OpModeType {
        TELEOP,
        AUTO
    }
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20240302_083301.tflite";
    private static final String[] LABELS = {
            "Pawn",
    };
    public Neptune(CommandOpMode opMode, OpModeType opModeType) {
        mOpMode = opMode;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(opMode.hardwareMap), false);
        outtake = new OutakeSubsystem(opMode.hardwareMap.get(Servo.class, "outtakeServo"));
        intake = new IntakeSubsystem(new MotorEx(opMode.hardwareMap, "intakeMotor", 537.6,340),
                (new MotorEx(opMode.hardwareMap, "intakeMotor2", 537.6,340)),
                opMode.hardwareMap.get(Servo.class, "intakeServo1"), opMode.hardwareMap.get(Servo.class, "intakeServo2"));
        slides = new SlidesSubsystem(this, new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312),
                opMode.hardwareMap.get(Servo.class, "vbarServo"),  opMode, opMode.hardwareMap.get(AnalogInput.class, "vbarAnalog"));
        driverOp = new GamepadEx(opMode.gamepad1);
        gunnerOp = new GamepadEx(opMode.gamepad2);
        hangServo = opMode.hardwareMap.get(Servo.class, "hangServo1");
        hangServo2 = opMode.hardwareMap.get(Servo.class, "hangServo2");
        hangMotor = new MotorEx(opMode.hardwareMap,"hangMotor", Motor.GoBILDA.RPM_312);
        launcher = new DroneLauncherSubsytem(opMode.hardwareMap.get(Servo.class, "droneLauncher"));
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
        indicatorSubsytem = new IndicatorSubsytem(this);
        int ledCount = 6;
        for(Integer i = 0; i< ledCount; i++)
        {
            indicatorSubsytem.registerLED(opMode.hardwareMap.get(LED.class, "led" + i.toString()), i % 2);
        }


        hang = new HangSubsystem(hangServo,hangServo2,hangMotor);

//        hangController = new PIDSlidesController(new SimpleLinearLift(hangServo));

        // register subsystems
        if (opModeType == OpModeType.AUTO){
            vision = new VisionSubsystem(opMode, TFOD_MODEL_ASSET, LABELS);
            opMode.register(vision);
        }

        opMode.register(drive);
        opMode.register(outtake);
        opMode.register(intake);
        opMode.register(slides);
        opMode.register(hang);
        opMode.register(launcher);

        // driver button setup
        intakeReverseButton = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        intakeButton = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);
        driveBrakeTrigger = new GamepadTrigger(driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER);
        hangArmButtonUp = new GamepadButton(driverOp, GamepadKeys.Button.X);
        hangButtonUp = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN);
        hangButtonDown = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP);


        // gunner button setup
        liftButton = new GamepadButton(gunnerOp, GamepadKeys.Button.X);
        liftButtonDown = new GamepadButton(gunnerOp, GamepadKeys.Button.Y);
        outtakeButton = new GamepadButton(gunnerOp, GamepadKeys.Button.A);
        droneLauncherButton = new GamepadButton(gunnerOp, GamepadKeys.Button.LEFT_BUMPER);
        droneLauncherButton2 = new GamepadButton(gunnerOp, GamepadKeys.Button.B);

        manualSlideButtonUp = new GamepadTriggerAsButton(gunnerOp, GamepadKeys.Trigger.LEFT_TRIGGER, NeptuneConstants.TRIGGER_THRESHOLD);
        manualSlideButtonDown = new GamepadTriggerAsButton(gunnerOp, GamepadKeys.Trigger.RIGHT_TRIGGER, NeptuneConstants.TRIGGER_THRESHOLD);

        // pseudo buttons
        magSwitchButton = new SwitchReader(opMode.hardwareMap, false);
        magSwitchButton.whileActiveContinuous(new InstantCommand(slides::stopMotorResetEncoder));
    }



        //Start positions for each auto placement
    public void setStartPosition(FieldPos fp, AllianceColor ac) {

        this.fieldPos = fp;
        this.allianceColor = ac;


        if (fp == FieldPos.BD){
            this.startPos = Trajectories.BDStart;
        }
        else if (fp == FieldPos.AU){
            this.startPos = Trajectories.AUStart;

        }

        if(ac == AllianceColor.BLUE){
            this.startPos = new Pose2d(this.startPos.getX(), -this.startPos.getY(), -this.startPos.getHeading());
        }

        drive.setPoseEstimate(this.startPos);
    }
}
