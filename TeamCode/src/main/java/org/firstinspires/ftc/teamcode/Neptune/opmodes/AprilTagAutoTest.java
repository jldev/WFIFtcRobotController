package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectAprilTagCommand;
import org.firstinspires.ftc.teamcode.Neptune.commands.DetectPawnCommand;
import org.firstinspires.ftc.teamcode.Neptune.drive.Trajectories;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@Autonomous(group = "drive", name = "April Tag Auto Test")
public class AprilTagAutoTest extends CommandOpMode {
    NeptuneAuto neptuneAuto;
    @Override
    public void initialize() {
        neptuneAuto = new NeptuneAuto(this, Neptune.FieldPos.AU, Neptune.AllianceColor.RED);
        neptuneAuto.trajectories.setupTrajectories(Trajectories.PropPlacement.CENTER);
        DetectAprilTagCommand detectAprilTagCommand = new DetectAprilTagCommand(neptuneAuto.neptune.vision, neptuneAuto.trajectories.targettedAprilTag);

        schedule(
            detectAprilTagCommand.whenFinished(() -> {
                AprilTagPoseFtc pose = detectAprilTagCommand.getPoseFromDetection();
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", pose.x, pose.y, pose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", pose.pitch, pose.roll, pose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", pose.range, pose.bearing, pose.elevation));
                telemetry.update();

            }));
    }
}
