package org.firstinspires.ftc.teamcode.G3GSS.Teleop;//package org.firstinspires.ftc.teamcode.GSS.G3GSS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FSM_teleop")
public class FSM_teleop extends OpMode {

    final int LIFT_HIGH = 3;
    final int LIFT_LOW = 0;
    final int ARM_ROTATE = 3;
    final int ARM_IDLE = 0;

    private DcMotor leftslide;
    private DcMotor rightslide;
    private DcMotor arm;
    private Servo claw;

    LiftState liftstate = LiftState.LIFT_START;

    ElapsedTime liftTimer = new ElapsedTime();

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_RETRACT,
        ARM_ROTATE
    }

    ;

    public void init() {
        liftTimer.reset();

        leftslide = hardwareMap.get(DcMotor.class, "leftslide");

        rightslide = hardwareMap.get(DcMotor.class, "rightslide");

        arm = hardwareMap.get(DcMotor.class, "arm");

        leftslide.setTargetPosition(LIFT_LOW);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightslide.setTargetPosition(LIFT_LOW);
        rightslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setTargetPosition(ARM_ROTATE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {
        leftslide.setPower(1.0);
        rightslide.setPower(1.0);

        switch (liftstate) {
            case LIFT_START:

                if (gamepad2.x) {
                    leftslide.setTargetPosition(LIFT_HIGH);
                    rightslide.setTargetPosition(LIFT_HIGH);
                    liftstate = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                if (Math.abs(leftslide.getCurrentPosition() - LIFT_HIGH) < 10) {
                    arm.setTargetPosition(ARM_ROTATE);

                    liftTimer.reset();
                    liftstate = LiftState.ARM_ROTATE;
                }
                break;
            case ARM_ROTATE:
                if (liftTimer.seconds() >= ARM_ROTATE) {
                    arm.setTargetPosition(ARM_IDLE);
                    leftslide.setTargetPosition(LIFT_LOW);
                    rightslide.setTargetPosition(LIFT_LOW);
                    liftstate = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(leftslide.getCurrentPosition() - LIFT_LOW) < 10) {
                    liftstate = LiftState.LIFT_START;
                }
                break;
            default:
                liftstate = LiftState.LIFT_START;


        }
        if (gamepad2.y && liftstate != LiftState.LIFT_START) {
            liftstate = LiftState.LIFT_START;
        }

    }

}



