package org.firstinspires.ftc.teamcode.Helix.opmodes;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Helix.subsystems.IntakeSubsystem;


//values
import static org.firstinspires.ftc.teamcode.Helix.subsystems.IntakeSubsystem.GripperState.OPEN;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private Helix helix;

    @Override
    public void initialize() {
        helix = new Helix(this, Helix.OpModeType.TELEOP, Helix.AllianceColor.RED);

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(

//                neptune.drive, () -> -neptune.driverOp.getLeftY(),
//                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,

                helix.drive, () -> helix.driverOp.getRightY(),
                () -> helix.driverOp.getRightX(), () -> helix.driverOp.getLeftX()
        );
        helix.drive.setDefaultCommand(driveCommand);




        //         INTAKE

        helix.instakeGripperButton.whileHeld(helix.intake.setGripperOpen());
        helix.instakeGripperButton.whenReleased(helix.intake.setGripperClosed());

        helix.intakeLiftButton.whenPressed(helix.intake.setLift());


    }

}
