package org.firstinspires.ftc.teamcode.Neptune;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Neptune_Teleop")
public class NeptuneTeleopField extends LinearOpMode {
    //    private static final boolean USE_WEBCAM = true;
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {


        Neptune neptune = new Neptune(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "initialized");
        telemetry.update();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = neptune.getRawExternalHeading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            neptune.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        }
    }
}