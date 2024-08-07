package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.G3GSS.GSTEST;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "rr_test")
public class rr_test extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

//        initAprilTag();
        waitForStart();


        GSTEST drive = new GSTEST(hardwareMap);

        telemetry.addData("Status", "initialized");
        telemetry.update();
        {
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(12)
                    .forward(50)
                    .strafeRight(24)
                    .back(50)
                    .strafeLeft(12)
                    .build();

            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(trajectory);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;

        }
    }
}

//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//
////                telemetryAprilTag();
//
//                // Push telemetry to the Driver Station.
//                telemetry.update();
//
//
//                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//                telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//                // Step through the list of detections and display info for each one.
//                for (AprilTagDetection detection : currentDetections) {
//                    if (detection.metadata != null) {
//                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                    } else {
//                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                    }
//                    double currentHeading = TESTG3S.getHeading();
//
//                    if (detection.ftcPose.range >= 12) {
//                        TESTG3S.driveStraight(0.3, 0.3, 300, currentHeading + detection.ftcPose.bearing, 3000);
//                    }
//
//
//                }   // end for() loop
//
//
//
//
//            }
//            visionPortal.close();
//      }
//
//
//    // Save more CPU resources when camera is no longer needed.
//
//
//}   // end method runOpMode()
//
//    /**
//     * Initialize the AprilTag processor.
//     */
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
//    }   // end method initAprilTag()

//    /**
//     * Add telemetry about AprilTag detections.
//     */
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }   // end method telemetryAprilTag()


