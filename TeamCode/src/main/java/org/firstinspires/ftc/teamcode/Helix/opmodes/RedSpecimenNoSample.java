package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helix.Helix;

@Config
@Autonomous(group = "drive", name = "Red Specimen No Sample")
public class RedSpecimenNoSample extends CommandOpMode {
    HelixAuto helixAuto;
    @Override
    public void initialize() {
        helixAuto = new HelixAuto(this, Helix.FieldPos.AU, Helix.AllianceColor.RED, Helix.Target.SPECIMENS);
        helixAuto.helix.pushSamples = false;
    }

    @Override
    public void run(){
        waitForStart();
        while (opModeIsActive()) {
            helixAuto.run();
            super.run();
        }

    }
}
