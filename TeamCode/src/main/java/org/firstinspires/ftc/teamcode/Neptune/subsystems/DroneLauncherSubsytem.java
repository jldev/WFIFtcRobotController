package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

public class DroneLauncherSubsytem extends SubsystemBase {

    public enum droneLaunchState {
        LAUNCH,
        NEUTRAL
    }

    private final Servo mDroneLauncherServo;

    public DroneLauncherSubsytem(Servo droneLauncherServo) {

       mDroneLauncherServo = droneLauncherServo;

       mDroneLauncherServo.setDirection(Servo.Direction.REVERSE);

       mDroneLauncherServo.setPosition(NeptuneConstants.DRONE_NEUTRAL);
    }

    droneLaunchState DroneLaunchState = droneLaunchState.LAUNCH;

    @Override
    public void periodic() {
        switch (DroneLaunchState) {
            case LAUNCH:
                mDroneLauncherServo.setPosition(NeptuneConstants.DRONE_LAUNCHED);
                break;
            case NEUTRAL:
                mDroneLauncherServo.setPosition(NeptuneConstants.DRONE_NEUTRAL);
                break;
        }

    }

    public void setDroneState (droneLaunchState state){
        DroneLaunchState = state;
    }

}
