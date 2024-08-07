package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

public class EndDistanceDriveCommand extends CommandBase {

    private final Neptune neptune;
    private final MecanumDriveSubsystem.DriveDirection direction;
    private final double endDistance;
    private final DistanceSensor distanceSensor;

    public EndDistanceDriveCommand(Neptune neptune, MecanumDriveSubsystem.DriveDirection direction, DistanceSensor sensor, double endDistance ) {
        this.neptune = neptune;
        this.direction = direction;
        this.distanceSensor = sensor;
        this.endDistance = endDistance;

    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute(){
        this.neptune.drive.driveDirection(direction, 0.8);
    }
    @Override
    public void end(boolean interrupted) {
        this.neptune.drive.stop();
    }

    @Override
    public boolean isFinished() {
        return distanceSensor.getDistance(DistanceUnit.INCH) < this.endDistance;
    }
}
