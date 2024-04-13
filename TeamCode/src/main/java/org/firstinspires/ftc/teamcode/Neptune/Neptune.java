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

    public final OpModeType mOpModeType;
    public VisionSubsystem vision;
    public final MecanumDriveSubsystem drive;
    public final OutakeSubsystem outtake;
    public final IntakeSubsystem intake;
    public final SlidesSubsystem slides;

    public DroneLauncherSubsytem launcher;

    public IndicatorSubsytem indicatorSubsytem;

    public HangSubsystem hang;
    public GamepadEx driverOp;
    public GamepadEx gunnerOp;
    public Servo hangServo;
    public Servo hangServo2;
    public MotorEx hangMotor;
    public GamepadButton liftButton;
    public GamepadButton liftButtonDown;
    public GamepadButton outtakeButton;
    public GamepadButton intakeButton;
    public GamepadButton hangButtonUp;
    public GamepadButton hangButtonDown;
    public GamepadButton hangArmButtonUp;
    public GamepadButton intakeReverseButton;

    public GamepadButton droneLauncherButton;

    public GamepadButton droneLauncherButton2;

    public final SwitchReader magSwitchButton;

    public final DistanceSensor distanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public GamepadTriggerAsButton manualSlideButtonUp;
    public GamepadTriggerAsButton manualSlideButtonDown;
    public GamepadTriggerAsButton intakeTrigger;
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
        mOpModeType = opModeType;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(opMode.hardwareMap), false);
        outtake = new OutakeSubsystem(opMode.hardwareMap.get(Servo.class, "outtakeServo"));
        intake = new IntakeSubsystem(this, new MotorEx(opMode.hardwareMap, "intakeMotor", 537.6,340),
                (new MotorEx(opMode.hardwareMap, "intakeMotor2", 537.6,340)),
                opMode.hardwareMap.get(Servo.class, "intakeServo1"), opMode.hardwareMap.get(Servo.class, "intakeServo2"));
        slides = new SlidesSubsystem(this, new MotorEx(opMode.hardwareMap, "slideMotor", Motor.GoBILDA.RPM_312),
                opMode.hardwareMap.get(Servo.class, "vbarServo"),  opMode, opMode.hardwareMap.get(AnalogInput.class, "vbarAnalog"));

        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // register subsystems
        if (opModeType == OpModeType.AUTO){
            vision = new VisionSubsystem(opMode, TFOD_MODEL_ASSET, LABELS);
            leftDistanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "leftDistance");
            rightDistanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "rightDistance");
            opMode.register(vision);
        } else {
            driverOp = new GamepadEx(opMode.gamepad1);
            gunnerOp = new GamepadEx(opMode.gamepad2);

            hangServo = opMode.hardwareMap.get(Servo.class, "hangServo1");
            hangServo2 = opMode.hardwareMap.get(Servo.class, "hangServo2");
            hangMotor = new MotorEx(opMode.hardwareMap,"hangMotor", Motor.GoBILDA.RPM_312);

            indicatorSubsytem = new IndicatorSubsytem(this);
            int ledCount = 6;
            for(int i = 0; i< ledCount; i++)
            {
                indicatorSubsytem.registerLED(opMode.hardwareMap.get(LED.class, "led" + Integer.toString(i)), i % 2);
            }

            launcher = new DroneLauncherSubsytem(opMode.hardwareMap.get(Servo.class, "droneLauncher"));
            hang = new HangSubsystem(hangServo,hangServo2,hangMotor);

            opMode.register(hang);
            opMode.register(launcher);
            opMode.register(indicatorSubsytem);

            // driver button setup
            intakeReverseButton = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
            intakeButton = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);
            intakeTrigger = new GamepadTriggerAsButton(driverOp, GamepadKeys.Trigger.LEFT_TRIGGER, 0.05);
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

        }

        opMode.register(drive);
        opMode.register(outtake);
        opMode.register(intake);
        opMode.register(slides);

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
