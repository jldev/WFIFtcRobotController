package org.firstinspires.ftc.teamcode.Helix;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helix.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Helix.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.Helix.subsystems.SlideSubsystem;

public class Helix {

    public final OpModeType mOpModeType;

    public final MecanumDriveSubsystem drive;
    //public final Limelight3A limelight;

    public GamepadEx driverOp;
    public GamepadEx gunnerOp;
    public Pose2d currentPos;



    //subsystems
    public final SlideSubsystem slides;
    public final PivotSubsystem pivot;
//    public final HangSubsystem hang;
    public final ClawSubsystem claw;


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


    // Gunner

    public GamepadButton instakeGripperButton;
    public GamepadButton intakeLiftButton;

    public GamepadButton verticleSlideUp;
    public GamepadButton verticleSlideDown;
    public GamepadButton horizontalSlideOut;
    public GamepadButton horizontalSlideIn;

    public GamepadButton home_slidePreset;
    public GamepadButton wall_slidePreset;
    public GamepadButton hang_slidePreset;
    public GamepadButton basket_slidePreset;


    // Driver

    public GamepadButton hangRaise;
    public GamepadButton hangLower;
    public GamepadButton pivotRaise;
    public GamepadButton pivotLower;

    public GamepadButton home_pivotPreset;
    public GamepadButton hang_pivotPreset;
    public GamepadButton basket_pivotPreset;
    public GamepadButton sub_pivotPreset;






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
//        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");



        //     slides
        slides = new SlideSubsystem(this,
                new MotorEx(opMode.hardwareMap, "slideVerticalMotor", Motor.GoBILDA.RPM_312),
                new MotorEx(opMode.hardwareMap, "slideHorizontalMotor", Motor.GoBILDA.RPM_1150),
                opMode,
                HelixConstants.SLIDES_PID_POS_COEFFICIENT,
                HelixConstants.SLIDES_PID_TOLERANCE
                );




        //     pivot
        pivot = new PivotSubsystem(this,
                new MotorEx(opMode.hardwareMap, "pivotMotor", Motor.GoBILDA.RPM_223),
                opMode,
                HelixConstants.SLIDES_PID_POS_COEFFICIENT,
                HelixConstants.SLIDES_PID_TOLERANCE
        );




        //     hang
//        hang = new HangSubsystem(this,
//                new MotorEx(opMode.hardwareMap, "hangMotor", Motor.GoBILDA.RPM_435),
//                opMode,
//                HelixConstants.SLIDES_PID_POS_COEFFICIENT,
//                HelixConstants.SLIDES_PID_TOLERANCE
//        );



        //     claw
        claw = new ClawSubsystem(this,
                opMode,
                opMode.hardwareMap.get(Servo.class, "yaw_1"),
                opMode.hardwareMap.get(Servo.class, "pitch_2"),
                opMode.hardwareMap.get(Servo.class, "grip_3"));





        opMode.register(drive);
        opMode.register(slides);
//        opMode.register(hang);
        opMode.register(pivot);
        opMode.register(claw);





        //       gunner setup

           //intake
        instakeGripperButton =  new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_RIGHT);
        intakeLiftButton =  new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_LEFT);     // these are temp - gunner's out of buttons

           //slide manual
        verticleSlideUp = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_UP);
        verticleSlideDown = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_DOWN);

        horizontalSlideOut = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_LEFT);
        horizontalSlideIn = new GamepadButton(gunnerOp, GamepadKeys.Button.DPAD_RIGHT);

           //slidePresets
        home_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.A);
        wall_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.X);
        hang_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.B);
        basket_slidePreset = new GamepadButton(gunnerOp, GamepadKeys.Button.Y);

          //claw
        // yaw = LTx
        // pitch = LTy
        // grip = RTy



        //     driver setup

           //hang
        hangRaise = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_LEFT);
        hangLower = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_RIGHT);
        // !!we should change this to have a button to go all the way up, and a button to go all the way down


           //pivot manual
        pivotRaise = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP);
        pivotLower = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN);

           //pivotPresets
        home_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.X);
        hang_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.B);
        basket_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.Y);
        sub_pivotPreset = new GamepadButton(driverOp, GamepadKeys.Button.A);


    }



    public void setStartPosition(Pose2d pos){
        this.currentPos = pos;
        drive.setPoseEstimate(this.currentPos);
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
