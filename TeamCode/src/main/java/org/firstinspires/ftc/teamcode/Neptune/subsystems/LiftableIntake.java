package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftableIntake extends SubsystemBase {
    public enum LiftableIntakePosition{
        POSITION_1,
        POSITION_2,
    };
    private final Servo mIntakeLiftServo;
    private final Motor mIntakeMotor;



    public LiftableIntake(Motor intakeMotor, Servo liftServo){
        mIntakeMotor = intakeMotor;
        mIntakeLiftServo = liftServo;
    }

    public void intakeOn(){
        mIntakeMotor.set(0.5);
    }

    public void intakeOff(){
        mIntakeMotor.stopMotor();
    }

    public void intakeReverse(){
        mIntakeMotor.set(-0.5);
    }
    public boolean intakeFull(){
        return false;
    }

    public void intakeLift(LiftableIntakePosition position){

    }
}
