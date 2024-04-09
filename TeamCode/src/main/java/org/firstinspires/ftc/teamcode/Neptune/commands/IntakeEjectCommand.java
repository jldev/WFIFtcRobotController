package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;

public class IntakeEjectCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeEjectCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {intake.setIntakeState(IntakeSubsystem.IntakeState.EJECTING);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
