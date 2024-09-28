package org.firstinspires.ftc.teamcode.Helix.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.subsystems.MecanumDriveSubsystem;

import java.util.List;

public class CenterOnSpecimenCommand extends CommandBase {
    private final Helix helix;
    private boolean done = false;

    public CenterOnSpecimenCommand(Helix helix) {
        this.helix = helix;
        addRequirements(helix.drive);
    }

    @Override
    public void execute() {
        LLResult result = helix.limelight.getLatestResult();
        if (result != null) {
            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

            for (LLResultTypes.ColorResult cr : colorResults) {
                double xAngle = cr.getTargetXDegrees();

                if (xAngle < 1.3f && xAngle > -1.3f) {
                    done = true;
                    continue;
                }

                //cant remember how to print :(

                //     "24" is distance from wall
                double driveDistance = 24 / Math.tan(Math.toRadians(90 - xAngle));

                if (!helix.drive.isBusy()) {
                    if (xAngle < 4.0f && xAngle > -4.0f) {
                        if (xAngle < 3.0f && xAngle > 0.0f) {
                            helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.RIGHT, .33f);
                        } else if (xAngle > -3.0f && xAngle < 0.0f) {
                            helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.LEFT, .33f);
                        }
                    } else {
                        helix.drive.driveDirection(MecanumDriveSubsystem.DriveDirection.RIGHT, driveDistance);
                    }
                }

                helix.mOpMode.telemetry.addLine("Drive Distance : " + driveDistance);
                helix.mOpMode.telemetry.update();

                helix.drive.waitForIdle();
                done = true;
                return;

            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        helix.drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || done;
    }
}
