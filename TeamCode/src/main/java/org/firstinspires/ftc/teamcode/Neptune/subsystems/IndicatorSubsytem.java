package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;

import java.util.ArrayList;

public class IndicatorSubsytem extends SubsystemBase {

    class IndicatorLED{
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



    private int TIME = 0;

    private double currentTime = 0;

    private double nextChange = 0.5;
    private final int nextState = 0;

    private  int currentState = 0;

    private Neptune mNeptune;

    private ArrayList<IndicatorLED> mleds = new ArrayList<>();






    public IndicatorSubsytem(Neptune neptune) {
        mNeptune = neptune;
    }

    public void registerLED(LED led, int color)
    {
        mleds.add(new IndicatorLED(led, color));
    }


    LEDState ledstate = LEDState.OFF;

    @Override
    public void periodic() {

       currentTime = mNeptune.mOpMode.getRuntime();

        if (currentTime > nextChange) {
            currentState = nextState;
            nextChange = (currentTime + nextChange);
            switch (ledstate) {
                case OFF:
                    for(IndicatorLED led : mleds){
                    led.led.enable(false);
                }
                    break;
                case RED:
                    for(IndicatorLED led : mleds){
                        if(led.color == 0)
                            led.led.enable(false);
                    }

                    break;
                case GREEN:
                    for(IndicatorLED led : mleds){
                        if(led.color == 1)
                            led.led.enable(false);
                    }

                    break;
            }
        }
    }

    public void setLedstate (IndicatorSubsytem state, int period){
//        nextState = state;
        nextChange = (TIME + period);
    }

    public void ledOFF(){
        ledstate = LEDState.OFF;
    }

    public void setLED (IndicatorSubsytem.LEDState state){
        ledstate = state;
    }


    public boolean ledturnedoff(){
        return false;
    }

}
