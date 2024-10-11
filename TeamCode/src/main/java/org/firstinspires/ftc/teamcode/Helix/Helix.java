package org.firstinspires.ftc.teamcode.Helix;

import android.transition.Slide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helix.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Helix.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.SlideSubsystem;

public class Helix {

    public final OpModeType mOpModeType;

    public final MecanumDriveSubsystem drive;
    public final Limelight3A limelight;

    public GamepadEx driverOp;
    public GamepadEx gunnerOp;
    public Pose2d currentPos;



    //subsystems
    public final IntakeSubsystem intake;
    public final SlideSubsystem slides;


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



    //           BUTTONSSSSS

    public GamepadButton instakeGripperButton;
    public GamepadButton intakeLiftButton;

    public GamepadButton tempSlideUpButton;
    public GamepadButton tempSlideDownButton;






    public enum OpModeType {
        TELEOP,
        AUTO
    }

    public Helix(CommandOpMode opMode, OpModeType opModeType, AllianceColor ac) {
        mOpMode = opMode;
        mOpModeType = opModeType;
        allianceColor = ac;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(opMode.hardwareMap), false);
        driverOp = new GamepadEx(opMode.gamepad1);
        gunnerOp = new GamepadEx(opMode.gamepad2);
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");


        //subsystems      - intake
        intake = new IntakeSubsystem(this,
                opMode.hardwareMap.get(Servo.class, "liftServo"),
                opMode.hardwareMap.get(Servo.class, "gripperServo"));



        //     slides
        slides = new SlideSubsystem(this,
                new MotorEx(opMode.hardwareMap, "slideVerticalMotor"),
                opMode,
                HelixConstants.SLIDES_PID_POS_COEFFICIENT,
                HelixConstants.SLIDES_PID_TOLERANCE
                );

        opMode.register(drive);
        opMode.register(intake);
        opMode.register(slides);





        //gunner setup
        instakeGripperButton =  new GamepadButton(gunnerOp, GamepadKeys.Button.X);
        intakeLiftButton =  new GamepadButton(gunnerOp, GamepadKeys.Button.B);
        tempSlideUpButton = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_UP);
        tempSlideDownButton = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_DOWN);
    }



        //Start positions for each auto placement
    public void setStartPosition(FieldPos fp, AllianceColor ac) {

        this.fieldPos = fp;
        this.allianceColor = ac;

        this.currentPos = new Pose2d(12, -62, Math.toRadians(90));

//        if (fp == FieldPos.BD){
//            this.startPos = Trajectories.BDStart;
//        }
//        else if (fp == FieldPos.AU){
//            this.startPos = Trajectories.AUStart;
//
//        }
//
//        if(ac == AllianceColor.BLUE){
//            this.startPos = new Pose2d(this.startPos.getX(), -this.startPos.getY(), -this.startPos.getHeading());
//        }

        drive.setPoseEstimate(this.currentPos);
    }
}
