package org.firstinspires.ftc.teamcode.Neptune;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Neptune_Teleop")
public class NeptuneTeleop  extends LinearOpMode {
    //    private static final boolean USE_WEBCAM = true;
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {


        Neptune Neptune = new Neptune(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "initialized");
        telemetry.update();

        while (opModeIsActive()) {

            double lefty = -gamepad1.left_stick_y;
            double righty = -gamepad1.right_stick_y;


            //Limiting the speed of the motors
            lefty = Range.clip(lefty, -0.85, 0.85);
            righty = Range.clip(righty, -0.85, 0.85);

            if (gamepad1.left_bumper) {
                Neptune.setMotorPowers(-.75, .75, -.75, .75);

            } else if (gamepad1.right_bumper) {
                Neptune.setMotorPowers(.75, -.75, .75, -.75);

            } else {
                Neptune.setMotorPowers(lefty,lefty, righty, righty );
            }

        }
    }
}