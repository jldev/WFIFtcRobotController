package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX, brakeTrigger;

    public MecanumDriveCommand(MecanumDriveSubsystem drive, DoubleSupplier leftY,
                               DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier brakeTrigger) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.brakeTrigger = brakeTrigger;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double damper = 1 - brakeTrigger.getAsDouble();
        drive.drive(leftY.getAsDouble() * damper, leftX.getAsDouble() * damper, rightX.getAsDouble() * damper);
    }

}