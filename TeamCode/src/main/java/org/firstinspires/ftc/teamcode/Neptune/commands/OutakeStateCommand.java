package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;
import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;

public class OutakeStateCommand extends CommandBase {
    private final OutakeSubsystem outake;
    private final OutakeSubsystem.OutakeState outakeState;

    public OutakeStateCommand(OutakeSubsystem outake, OutakeSubsystem.OutakeState outakeState) {
        this.outake = outake;
        this.outakeState = outakeState;

        addRequirements(outake);
    }

    @Override
    public void initialize() {outake.setOutakeState(this.outakeState);}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
