package org.firstinspires.ftc.teamcode.Helix.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helix.Helix;
import org.firstinspires.ftc.teamcode.Helix.Helix;

@Config
@Autonomous(group = "drive", name = "Test Auto")
public class TestAuto extends CommandOpMode {
    HelixAuto helixAUto;
    @Override
    public void initialize() {

        helixAUto = new HelixAuto(this, Helix.FieldPos.AU, Helix.AllianceColor.RED);
        helixAUto.run();
    }
}
