package org.firstinspires.ftc.teamcode.G3GSS.Teleop;
//

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.G3GSS.G3GSS;
import org.firstinspires.ftc.teamcode.G3GSS.Logger;
@Disabled
@TeleOp (name = "G3GSS_Teleop")
public class G3GSS_Teleop extends LinearOpMode {

    org.firstinspires.ftc.teamcode.G3GSS.G3GSS G3GSS = new G3GSS(this, new Logger(true, telemetry));

    @Override
    public void runOpMode() throws InterruptedException {

        G3GSS.init(hardwareMap, true);
        telemetry.addData("Status", "initilaized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            //Joystick directions on the gamepad
            double lefty = -gamepad1.left_stick_y;
            double righty = -gamepad1.right_stick_y;

            double lefty2 = -gamepad1.left_stick_y;
            double righty2 = -gamepad1.right_stick_y;


            //Limiting the speed of the motors
            lefty = Range.clip(lefty, -0.85, 0.85);
            righty = Range.clip(righty, -0.85, 0.85);

            lefty2 = Range.clip(lefty2, -0.85, 0.85);
            righty2 = Range.clip(righty2, -0.85, 0.85);


//                TODO: GAMEPAD 1 CONTROLS

            if (gamepad1.left_bumper) {
                G3GSS.leftRear.setPower(.75);
                G3GSS.rightRear.setPower(-.75);
                G3GSS.rightFront.setPower(.75);
                G3GSS.leftFront.setPower(-.75);
            } else if (gamepad1.right_bumper) {
                G3GSS.leftRear.setPower(-.75);
                G3GSS.rightRear.setPower(.75);
                G3GSS.rightFront.setPower(-.75);
                G3GSS.leftFront.setPower(.75);
            } else {
                G3GSS.leftFront.setPower(lefty);
                G3GSS.rightFront.setPower(righty);
                G3GSS.leftRear.setPower(lefty);
                G3GSS.rightRear.setPower(righty);
            }


            if (gamepad1.dpad_up) {
                G3GSS.leftRear.setPower(.8);
                G3GSS.leftFront.setPower(.8);
                G3GSS.rightFront.setPower(.8);
                G3GSS.rightRear.setPower(.8);
            } else if (gamepad1.dpad_down) {
                G3GSS.leftRear.setPower(-.8);
                G3GSS.leftFront.setPower(-.8);
                G3GSS.rightFront.setPower(-.8);
                G3GSS.rightRear.setPower(-.8);
            } else if (gamepad1.dpad_right) {
                G3GSS.leftRear.setPower(.8);
                G3GSS.rightRear.setPower(-.8);
                G3GSS.rightFront.setPower(-.8);
                G3GSS.leftFront.setPower(.8);
            } else if (gamepad1.dpad_left) {
                G3GSS.leftRear.setPower(-.8);
                G3GSS.rightRear.setPower(.8);
                G3GSS.rightFront.setPower(.8);
                G3GSS.leftFront.setPower(-.8);
            }


//                  TODO: GAMEPAD 2 CONTROLS (RUN_TO_POSITION)

            //Arm

//            if (G3GSS.armstop.getState() == false   ) {
//                G3GSS.arm.setTargetPosition(0);
//                G3GSS.arm.setPower(1.0);
//                while (G3GSS.arm.isBusy()) {
//                    // Wait for the motor to reach its target position
//                }
//                G3GSS.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                G3GSS.arm.setPower(0);
//            }
//

//            //Low junction
//            if (gamepad2.a) {
//                G3GSS.arm.setTargetPosition(700);
//            }
//            //Tall/Medium front
//            else if (gamepad2.b) {
//                G3GSS.arm.setTargetPosition(1200);
//            }
//            //Tall/Medium back
//            else if (gamepad2.y) {
//                G3GSS.arm.setTargetPosition(1900);
//            }
//            //Reset
//            else if (gamepad2.right_bumper && G3GSS.armstop.getState() == true) {
//                G3GSS.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                G3GSS.arm.setPower(0.8);
//            }
//            else if (G3GSS.armstop.getState() == false){
//                G3GSS.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                G3GSS.arm.setTargetPosition(0);
//                G3GSS.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                G3GSS.arm.setPower(0.9);
//            }





//                  TODO: MANUAL GAMEPAD 2 CONTROLS

            //Slides
            if (gamepad2.dpad_up) {
                G3GSS.leftslide.setPower(0.9);
                G3GSS.rightslide.setPower(0.9);
            }
            else if (gamepad2.dpad_down && G3GSS.magswitch.getState() == true) {
                G3GSS.leftslide.setPower(-0.9);
                G3GSS.rightslide.setPower(-0.9);
            }
            else if (G3GSS.magswitch.getState() == false) {
                G3GSS.leftslide.setPower(0);
                G3GSS.rightslide.setPower(0);
            }
            else {
                G3GSS.leftslide.setPower(0);
                G3GSS.rightslide.setPower(0);
            }
//
//          //  Arm
            if (gamepad2.right_bumper) {
                G3GSS.arm.setPower(-.80);
            } else if (gamepad2.left_bumper && G3GSS.armstop.getState() == true) {
                G3GSS.arm.setPower(.80);
            } else {
                G3GSS.arm.setPower(0);
            }
//
//
//            //Servo for the claw
            if (gamepad2.x) {
                G3GSS.claw.setPosition(0.6);
            }
            else if (gamepad2.dpad_left) {
                G3GSS.claw.setPosition(0);
            }
            else {
                G3GSS.claw.setPosition(0.1);
            }


//                  TODO: TELEMETRY UPDATES

            telemetry.addData("Blue Value", G3GSS.colordetection.blue());
            telemetry.addData("Red Value", G3GSS.colordetection.red());
            telemetry.addData("Green Value", G3GSS.colordetection.green());
            telemetry.addData("armstop", G3GSS.armstop.getState());
            telemetry.addData("magswitch", G3GSS.magswitch.getState());
            telemetry.addData("leftslide", G3GSS.leftslide.getCurrentPosition());
            telemetry.addData("rightslide", G3GSS.rightslide.getCurrentPosition());
//            telemetry.addData("leftFrontPos", G3GSS.leftFront.getCurrentPosition());
//            telemetry.addData("LeftRearPos", G3GSS.leftRear.getCurrentPosition());
//            telemetry.addData("rightFrontPos", G3GSS.rightFront.getCurrentPosition());
//            telemetry.addData("leftRearPos",G3GSS.leftRear.getCurrentPosition());
            telemetry.addData("arm", G3GSS.arm.getCurrentPosition());
//            telemetry.addData("gyro", G3GSS.getHeading());
            telemetry.update();

        }
    }
}
