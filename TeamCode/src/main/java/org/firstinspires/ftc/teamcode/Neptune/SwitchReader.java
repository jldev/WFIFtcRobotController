package org.firstinspires.ftc.teamcode.Neptune;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

public class SwitchReader extends Button {

    public DigitalChannel magswitch = null;

    private BooleanSupplier state = null;


    public SwitchReader(HardwareMap hwMap){
        magswitch = hwMap.digitalChannel.get("magswitch");
        magswitch.setMode(DigitalChannel.Mode.INPUT);


        state = () -> magswitch.getState();
    }


}
