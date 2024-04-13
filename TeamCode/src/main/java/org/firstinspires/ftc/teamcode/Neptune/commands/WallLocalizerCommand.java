package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

public class WallLocalizerCommand extends CommandBase {

    private final Neptune neptune;
    private final MecanumDriveSubsystem.DriveDirection direction;
    private final double endDistance;
    private final DistanceSensor distanceSensor;

    private final double inchesThreshold = 1.0;

    public WallLocalizerCommand(Neptune neptune, MecanumDriveSubsystem.DriveDirection direction, DistanceSensor sensor, double endDistance ) {
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
        double delta = endDistance - distanceSensor.getDistance(DistanceUnit.INCH);
        this.neptune.drive.driveDirection(direction, delta/2);
    }
    @Override
    public void end(boolean interrupted) {
        this.neptune.drive.stop();
    }

    @Override
    public boolean isFinished() {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        return  (currentDistance < (this.endDistance + inchesThreshold) &&
                currentDistance > (this.endDistance - inchesThreshold));
    }
}
