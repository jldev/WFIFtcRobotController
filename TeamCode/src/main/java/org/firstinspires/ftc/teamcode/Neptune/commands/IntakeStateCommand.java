package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;

public class IntakeStateCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final IntakeSubsystem.IntakeState intakeState;

    public IntakeStateCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeState intakeState) {
        this.intake = intake;
        this.intakeState = intakeState;

        addRequirements(intake);
    }

    @Override
    public void initialize() {intake.setIntakeState(this.intakeState);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
