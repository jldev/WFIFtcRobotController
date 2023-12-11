package org.firstinspires.ftc.teamcode.Neptune;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Config
@Autonomous(group = "drive")
public class RedRightAuto extends LinearOpMode {

    private static final String LEFT = "Left";
    private static final String RIGHT = "Right";
    private static final String CENTER = "Center";

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231127_183907.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pawn",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
x     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        Neptune drive = new Neptune(hardwareMap);

        initTfod();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                String pawnLocation;
                double startTime = getRuntime();
                while (getPawnLocation() == null && opModeIsActive() && (getRuntime() - startTime) > 2){
                    sleep(50);
                }
                pawnLocation = getPawnLocation();
                if (pawnLocation == null ){
                    pawnLocation = LEFT;
                }

                telemetry.addData("Pawn Location:", pawnLocation);
                telemetry.update();


                //     The position we start at
                Pose2d start = new Pose2d(-12, 62, Math.toRadians(90));

                //     The position we go to after locating the team prop, lining up to go to the correct spike mark
                Pose2d initialSpike = new Pose2d(-12, 46, Math.toRadians(90));

                //     The left, center, and right spike mark locations
                Pose2d leftSpike = new Pose2d(0, 35, Math.toRadians(135));
                Pose2d centerSpike = new Pose2d(-12, 32, Math.toRadians(90));
                Pose2d rightSpike = new Pose2d(-12, 29, Math.toRadians(45));

                //     The left, center, and right backdrop locations
                Pose2d leftBackdrop = new Pose2d(-50, 28, Math.toRadians(0));
                Pose2d centerBackdrop = new Pose2d(-50, 35, Math.toRadians(0));
                Pose2d rightBackdrop = new Pose2d(-50, 42, Math.toRadians(0));

                //     The point in between the backdrop and stack, to help guide the robot
                Pose2d stageIn = new Pose2d(0, 0, Math.toRadians(0));
                Pose2d stageOut = new Pose2d(0, 0, Math.toRadians(0));

                //     The position we go to after we deliver a pixel, where we choose which stack to go to. We either go through the inside or outside trusses
                Pose2d initialStackIn = new Pose2d(0, 0, Math.toRadians(0));
                Pose2d initialStackOut = new Pose2d(0, 0, Math.toRadians(0));

                //     The locations we need to collect from the left, center, and right stacks
                Pose2d leftStack = new Pose2d(0, 0, Math.toRadians(0));
                Pose2d centerStack = new Pose2d(0, 0, Math.toRadians(0));
                Pose2d rightStack = new Pose2d(60, 12, Math.toRadians(0));

                drive.setPoseEstimate(start);

                //     The needed locations, based on where the team prop is
                Pose2d neededSpike = centerSpike;
                Pose2d neededBackdrop = centerBackdrop;



                //     We've located the team prop, and are now driving to the spike mark, then backdrop.
                if(pawnLocation.equals(LEFT))
                {
                    neededSpike = leftSpike;
                    neededBackdrop = leftBackdrop;
                } else if(pawnLocation.equals(CENTER))
                {
                    neededSpike = centerSpike;
                    neededBackdrop = centerBackdrop;
                } else if(pawnLocation.equals(RIGHT))
                {
                    neededSpike = rightSpike;
                    neededBackdrop = rightBackdrop;
                }


                //     A chain of positions to drive to
                Trajectory initialTraj = drive.trajectoryBuilder(start, true)
                        .lineToSplineHeading(initialSpike)
                        .build();

                Trajectory placePixelTraj = drive.trajectoryBuilder(initialSpike, true)
                        .lineToSplineHeading(neededSpike)
                        .build();

                Trajectory driveToBackboardTraj = drive.trajectoryBuilder(neededSpike, true)
                        .lineToSplineHeading(neededBackdrop)
                        .build();

                 Trajectory driveToStageIn = drive.trajectoryBuilder(neededBackdrop, true)
                        .lineToSplineHeading(stageIn)
                        .build();

                Trajectory driveToRightStack = drive.trajectoryBuilder(stageIn, true)
                        .lineToSplineHeading(rightStack)
                        .build();



                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("finalX", poseEstimate.getX());
                telemetry.addData("finalY", poseEstimate.getY());
                telemetry.addData("finalHeading", poseEstimate.getHeading());

                telemetry.update();




                if (isStopRequested()) return;

                drive.followTrajectory(initialTraj);
                drive.followTrajectory(placePixelTraj);
                drive.followTrajectory(driveToBackboardTraj);
                drive.followTrajectory(driveToStageIn);
                drive.followTrajectory(driveToRightStack);

                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("finalX", poseEstimate.getX());
                telemetry.addData("finalY", poseEstimate.getY());
                telemetry.addData("finalHeading", poseEstimate.getHeading());
                telemetry.update();



                sleep(5000);
            }
        }
                visionPortal.close();
    }

    private String getPawnLocation() {



            List<Recognition> currentRecognitions = tfod.getRecognitions();

//            telemetry.addData("# Objects Detected", currentRecognitions.size());

            if (currentRecognitions.size() > 0) {
                Recognition recognition = currentRecognitions.get(0);

                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.update();
                if (x > 400) { // - && y > 200 - this may not work
                    return RIGHT;
                } else if (x < 100) {
                    return LEFT;
                } else {
                    return CENTER;
                }

            }

        return null;
    }


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
//            .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);


        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }

}
