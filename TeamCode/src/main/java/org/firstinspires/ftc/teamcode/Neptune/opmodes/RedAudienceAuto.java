package org.firstinspires.ftc.teamcode.Neptune.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;

@Config
@Autonomous(group = "drive", name = "Red Audience Auto")
public class RedAudienceAuto extends CommandOpMode {
    NeptuneAuto neptuneAuto;
    @Override
    public void initialize() {
        neptuneAuto = new NeptuneAuto(this, Neptune.FieldPos.AU, Neptune.AllianceColor.RED);
        neptuneAuto.run();
    }
}
