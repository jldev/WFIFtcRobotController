package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

public class BDDriveCommand extends CommandBase {

    private final Neptune neptune;
    private final MecanumDriveSubsystem.DriveDirection direction;
    private final double distanceInches;


    public BDDriveCommand(Neptune neptune, MecanumDriveSubsystem.DriveDirection direction, double inches ) {
        this.neptune = neptune;
        this.direction = direction;
        this.distanceInches = inches;

    }

    @Override
    public void initialize() {

        this.neptune.drive.driveDirection(direction, distanceInches);}


    @Override
    public void end(boolean interrupted) {
        this.neptune.drive.stop();
    }

    @Override
    public boolean isFinished() {
        return neptune.distanceSensor.getDistance(DistanceUnit.INCH) < distanceInches;
    }
}
