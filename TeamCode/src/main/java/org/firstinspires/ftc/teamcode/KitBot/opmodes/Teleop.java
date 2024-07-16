package org.firstinspires.ftc.teamcode.KitBot.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KitBot.KitBot;
import org.firstinspires.ftc.teamcode.KitBot.commands.MecanumDriveCommand;

@TeleOp(name = "Teleop")
public class Teleop extends CommandOpMode {

    private KitBot kitBot;

    @Override
    public void initialize() {
        kitBot = new KitBot(this, KitBot.OpModeType.TELEOP, KitBot.AllianceColor.RED);

        // Drive control
        MecanumDriveCommand driveCommand = new MecanumDriveCommand(

//                neptune.drive, () -> -neptune.driverOp.getLeftY(),
//                neptune.driverOp::getLeftX, neptune.driverOp::getRightX,

                kitBot.drive, () -> kitBot.driverOp.getRightY(),
                () -> kitBot.driverOp.getRightX(), () -> kitBot.driverOp.getLeftX()
        );
        kitBot.drive.setDefaultCommand(driveCommand);

    }

}
