package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraControls;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Neptune.Neptune;
import org.firstinspires.ftc.teamcode.Neptune.NeptuneConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSubsystem  extends SubsystemBase {

    public enum VisionMode{
        TENSORFLOW,
        APRIL_TAG,
    }
    private VisionMode mode = VisionMode.TENSORFLOW;

    private final String[] labels;
    private final String tfodAssetName;
    private final CameraName cameraName;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     x     */
    private TfodProcessor tfod;

    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private List<Recognition> recognitions;
    private List<AprilTagDetection> detections;

    private CommandOpMode opMode;
    public VisionSubsystem(CommandOpMode om, String tensorflowModelAsset, String[] labels){
        this.opMode = om;
        this.labels = labels;
        this.tfodAssetName = tensorflowModelAsset;
        this.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        init();
    }

    @Override
    public void periodic(){
        switch (mode){
            case TENSORFLOW:
                if (!visionPortal.getProcessorEnabled(tfod)){
                    visionPortal.setProcessorEnabled(aprilTag, false);
                    visionPortal.setProcessorEnabled(tfod, true);
                }
                this.recognitions = tfod.getRecognitions();
                break;
            case APRIL_TAG:
                if (!visionPortal.getProcessorEnabled(aprilTag)){
                    visionPortal.setProcessorEnabled(aprilTag, true);
                    visionPortal.setProcessorEnabled(tfod, false);
                }
                this.detections = aprilTag.getDetections();
                break;
        }

    }

//    public void detectTensorFlowObject(){
//        this.setMode(VisionMode.TENSORFLOW);
//        this.start();
//    }
//
//    public void detectAprilTags(){
//        this.setMode(VisionMode.APRIL_TAG);
//        this.start();
//    }
    public void setMode(VisionSubsystem.VisionMode mode){
        this.mode = mode;
        start();
    }
    public double[] getCenterOfRecognition(Recognition recognition){
        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;
        return new double[]{x,y};
    }
    public List<Recognition> getRecognitions(){
        return this.recognitions;
    }

    private void setExposureAndGain(long exposureMS, int gain){

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
        }

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void init() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(this.tfodAssetName)
                .setModelLabels(this.labels)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(this.cameraName);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.

        builder.addProcessor(tfod);
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

//        setExposureAndGain((long)NeptuneConstants.CAMERA_EXPOSURE_TIME_MS, NeptuneConstants.CAMERA_GAIN);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(aprilTag, false);

    }

    public List<AprilTagDetection> getAprilTagDetections(){
        return this.detections;
    }

    public void start(){
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            visionPortal.resumeStreaming();
        }
    }
    public void stop() {
//        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            visionPortal.stopStreaming();
//        }
    }

    public void addTelemetry(Telemetry telemetry){
        switch(mode){
            case TENSORFLOW:
                telemetry.addData("# AprilTags Detected", this.recognitions.size());
                if (this.recognitions.size() > 0) {
                    Recognition recognition = this.recognitions.get(0);
                    double pos[] = getCenterOfRecognition(recognition);
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", pos[0], pos[1]);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                }
                break;
            case APRIL_TAG:
                telemetry.addData("# AprilTags Detected", this.detections.size());

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : this.detections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }   // end for() loop
                break;
        }
    }
}
