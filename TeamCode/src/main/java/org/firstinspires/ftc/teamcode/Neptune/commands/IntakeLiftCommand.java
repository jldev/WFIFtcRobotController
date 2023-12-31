package org.firstinspires.ftc.teamcode.Neptune.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Neptune.subsystems.IntakeSubsystem;

public class IntakeLiftCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final IntakeSubsystem.LiftableIntakePosition intakeLiftState;

    public IntakeLiftCommand(IntakeSubsystem intake, IntakeSubsystem.LiftableIntakePosition liftstate) {
        this.intake = intake;
        this.intakeLiftState = liftstate;

        addRequirements(intake);
    }

    @Override
    public void initialize() {intake.setIntakeLiftState(this.intakeLiftState);

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
