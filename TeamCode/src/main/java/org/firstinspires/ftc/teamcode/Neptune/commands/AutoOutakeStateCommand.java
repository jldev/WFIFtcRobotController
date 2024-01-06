package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.OutakeSubsystem;

public class AutoOutakeStateCommand extends CommandBase {
    private final OutakeSubsystem outake;
    private final OutakeSubsystem.AutoOutakeState autoOutakeState;

    public AutoOutakeStateCommand(OutakeSubsystem outake, OutakeSubsystem.AutoOutakeState autoOutakeState) {
        this.outake = outake;
        this.autoOutakeState = autoOutakeState;

        addRequirements(outake);
    }

    @Override
    public void initialize() {outake.setAutoOutakeState(this.autoOutakeState);}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
