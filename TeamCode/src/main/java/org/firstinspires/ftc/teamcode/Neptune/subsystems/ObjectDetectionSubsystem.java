package org.firstinspires.ftc.teamcode.Neptune.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class ObjectDetectionSubsystem  extends SubsystemBase {
    private final String[] labels;
    private final String tfodAssetName;
    private final CameraName cameraName;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     x     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private List<Recognition> recognitions;

    public ObjectDetectionSubsystem(CameraName camera, String tensorflowModelAsset, String[] labels){
        this.labels = labels;
        this.tfodAssetName = tensorflowModelAsset;
        this.cameraName = camera;

        initTfod();
    }

    public void detect(){
        this.recognitions = tfod.getRecognitions();
    }

    public double[] getCenterOfRecognition(Recognition recognition){
        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;
        return new double[]{x,y};
    }
    public List<Recognition> getRecognitions(){
        return this.recognitions;
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(this.tfodAssetName)
                .setModelLabels(this.labels)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(this.cameraName);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 360));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public void stop() {
        visionPortal.close();
    }
}
