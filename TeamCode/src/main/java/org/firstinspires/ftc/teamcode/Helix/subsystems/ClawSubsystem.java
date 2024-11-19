package org.firstinspires.ftc.teamcode.Helix.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helix.Helix;

public class ClawSubsystem extends SubsystemBase {

    private final Helix mHelix;

    private final CommandOpMode mOpMode;

    private final Servo yaw;
    private final Servo pitch;
    private final Servo grip;



    public ClawSubsystem(Helix helix, CommandOpMode commandOpMode, Servo yaw_1, Servo pitch_2, Servo grip_3)
    {
        mHelix = helix;
        mOpMode = commandOpMode;

        yaw = yaw_1;
        pitch = pitch_2;
        grip = grip_3;

        yaw.setPosition(.5);
        pitch.setPosition(.5);
    }

    @Override
    public void periodic()
    {
        double desiredYaw;
        double desiredPitch;

//        desiredYaw = (mHelix.gunnerOp.getLeftX() / 2) + .5;
//        desiredPitch = (mHelix.gunnerOp.getLeftY() / 2) + .5;


        desiredYaw = yaw.getPosition() + mHelix.gunnerOp.getLeftX() * .045;
        desiredPitch = pitch.getPosition() + mHelix.gunnerOp.getLeftY() * .045;

        if(desiredYaw > 1.00)
            desiredYaw = 1.00;
        if(desiredYaw < 0.00)
            desiredYaw = 0.00;

        if(desiredPitch > 1.00)
            desiredPitch = 1.00;
        if(desiredPitch < 0.00)
            desiredPitch = 0.00;



        mOpMode.telemetry.addData("Yaw:" , desiredYaw);
        mOpMode.telemetry.addData("Pitch:" , desiredPitch);
        mOpMode.telemetry.update();


        yaw.setPosition(desiredYaw);
        pitch.setPosition(desiredPitch);


        if(mHelix.gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .3)
        {
            grip.setPosition(0);
        } else
        {
            grip.setPosition(1);
        }
    }
}
