package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Helix.drive.Trajectories;

public class HelixAuto {

    public final Helix helix;
    public final Trajectories trajectories;

    private boolean mStacks = false;
    private final CommandOpMode opMode;

    public HelixAuto(CommandOpMode commandOpMode, Helix.FieldPos startingPosition, Helix.AllianceColor allianceColor, boolean stacks) {
        this(commandOpMode, startingPosition, allianceColor);
        mStacks = stacks;
    }

    public HelixAuto(CommandOpMode commandOpMode, Helix.FieldPos startingPosition, Helix.AllianceColor allianceColor) {
        opMode = commandOpMode;
        helix = new Helix(opMode, Helix.OpModeType.AUTO, allianceColor);
        helix.setStartPosition(startingPosition, allianceColor);
        trajectories = new Trajectories(helix);
    }

    private SequentialCommandGroup driveTest(Trajectories trajectories) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> helix.intake.setLift(.2)),
                new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(trajectories.waypoint1)),
                new TrajectoryFollowerCommand(helix.drive, trajectories.getTrajectory(trajectories.waypoint2))
                );
    }

    public void run() {
        opMode.schedule(driveTest(trajectories));
    }
}
