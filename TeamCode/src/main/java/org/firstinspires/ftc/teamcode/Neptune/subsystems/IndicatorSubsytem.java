package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

import java.util.ArrayList;

public class IndicatorSubsytem extends SubsystemBase {

    private final int RED = 0;
    private final int GREEN = 1;

    static class IndicatorLED{
        LED led;
        int color;
        public IndicatorLED(LED led, int color)
        {
            this.led = led;
            this.color = color;
        }
    };

    public enum LEDState {
        OFF,
        RED,
        GREEN
    }

    private final Neptune mNeptune;

    private final ArrayList<IndicatorLED> mleds = new ArrayList<>();

    public IndicatorSubsytem(Neptune neptune) {
        mNeptune = neptune;
    }

    public void registerLED(LED led, int color)
    {
        mleds.add(new IndicatorLED(led, color));
    }

    LEDState currentState = LEDState.OFF;

    @Override
    public void periodic() {
        if (mNeptune.slides.mVBarCurrentPosition == SlidesSubsystem.VBarPosition.UP){
            double currentDistance = mNeptune.distanceSensor.getDistance(DistanceUnit.INCH);
            if (currentDistance > 12){ // TODO: This needs to be a constant
                currentState = LEDState.OFF;
            } else if ((currentDistance < 12) && (currentDistance > 3)) {
                currentState = LEDState.GREEN;
            }else{
                currentState = LEDState.RED;
            }
            for(IndicatorLED led : mleds){
                switch (currentState) {
                    case OFF:
                        led.led.enable(false);
                        break;
                    case RED:
                        led.led.enable(led.color == RED);
                        break;
                    case GREEN:
                        led.led.enable(led.color == GREEN );
                        break;
                }
            }

        } else {
            // turn all leds off when vbar is not up
            for(IndicatorLED led : mleds){
                led.led.enable(false);
            }
        }
    }
}
