package org.firstinspires.ftc.teamcode.Neptune.commands;

import android.hardware.Sensor;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.MecanumDriveSubsystem;

public class WallLocalizerCommand extends CommandBase {

    private final Neptune neptune;
    private final MecanumDriveSubsystem.DriveDirection direction;
    private final double endDistance;
    private final DistanceSensor distanceSensor;

    private final double inchesThreshold = 3.0;

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
        double currentDistance = DistanceSensor.distanceOutOfRange;
        while(currentDistance > 300){
            currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        }
        double delta = currentDistance - endDistance;
//        neptune.mOpMode.telemetry.addData("Wall Driving Distance = ", delta);
//        neptune.mOpMode.telemetry.update();
        this.neptune.drive.driveDirection(direction, delta);
    }
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
//        return true;
        double currentDistance = DistanceSensor.distanceOutOfRange;
        while(currentDistance > 300){
            currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        }
        return  (currentDistance < (this.endDistance + inchesThreshold) &&
                currentDistance > (this.endDistance - inchesThreshold));
    }
}
