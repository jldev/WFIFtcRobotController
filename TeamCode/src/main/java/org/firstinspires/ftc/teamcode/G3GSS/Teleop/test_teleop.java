package org.firstinspires.ftc.teamcode.G3GSS.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.G3GSS.TESTG3S;

@TeleOp(name = "test_teleop")
public class test_teleop  extends LinearOpMode {
//    private static final boolean USE_WEBCAM = true;
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
    private boolean hookstate = false;

    @Override
    public void runOpMode() throws InterruptedException {

//        initAprilTag();



        TESTG3S TESTG3S = new TESTG3S(this);
        TESTG3S.init(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "initialized");
        telemetry.update();

        while (opModeIsActive()) {

            double lefty = -gamepad1.left_stick_y;
            double righty = -gamepad1.right_stick_y;

            double lefty2 = -gamepad1.left_stick_y;
            double righty2 = -gamepad1.right_stick_y;


            //Limiting the speed of the motors
            lefty = Range.clip(lefty, -0.85, 0.85);
            righty = Range.clip(righty, -0.85, 0.85);

            lefty2 = Range.clip(lefty2, -0.85, 0.85);
            righty2 = Range.clip(righty2, -0.85, 0.85);

            if (gamepad1.left_bumper) {
                TESTG3S.left2.setPower(.75);
                TESTG3S.right2.setPower(-.75);
                TESTG3S.right1.setPower(.75);
                TESTG3S.left1.setPower(-.75);
            } else if (gamepad1.right_bumper) {
                TESTG3S.left2.setPower(-.75);
                TESTG3S.right2.setPower(.75);
                TESTG3S.right1.setPower(-.75);
                TESTG3S.left1.setPower(.75);
            } else {
                TESTG3S.left1.setPower(lefty);
                TESTG3S.right1.setPower(righty);
                TESTG3S.left2.setPower(lefty);
                TESTG3S.right2.setPower(righty);
            }

            if(gamepad1.a ) {
                hookstate = true;
                telemetry.addData("Status", "hookActivated");
                telemetry.update();
            }
            if (hookstate == true) {
                TESTG3S.hook.setPosition(0);
            }


        }
    }

//    private void initAprilTag() {
//
//        // Create the AprilTag processor the easy way.
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        // Create the vision portal the easy way.
//        if (USE_WEBCAM) {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
//        } else {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    BuiltinCameraDirection.BACK, aprilTag);
//        }
//
//    }

}