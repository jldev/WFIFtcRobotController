package org.firstinspires.ftc.teamcode.KitBot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.KitBot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.KitBot.drive.Trajectories;
import org.firstinspires.ftc.teamcode.KitBot.subsystems.MecanumDriveSubsystem;

public class KitBot {

    public final OpModeType mOpModeType;

    public final MecanumDriveSubsystem drive;

    public GamepadEx driverOp;
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

    public KitBot(CommandOpMode opMode, OpModeType opModeType, AllianceColor ac) {
        mOpMode = opMode;
        mOpModeType = opModeType;
        allianceColor = ac;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(opMode.hardwareMap), false);
        driverOp = new GamepadEx(opMode.gamepad1);
        opMode.register(drive);
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
