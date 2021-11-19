package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
public class PurpleTensorFlow {
    HardwareMap hwMap;
    private static final String VUFORIA_KEY =
            "AUEPTsj/////AAABmXYBynyLn0xeoNEKiUWEeAAEFvPFDHQUede2OVPhDHFAc4ZnvLxoHoluAS1ACHCMNJb6yYl3NuiHQmRc1m28p9sBRWxOxAEQluIxAP5botlaeikGtcPKmaQdcp98t53w3/WPnVC4OW9VAd6LD+8KFURWHmBm8RbqcCD+VOTenN3excKg8QuGrgiwgp2f21Hse0tkj02caYZovIUxyodab9PHydO0FbvjinBbRcPoh4zN/YmV0IRRrUaxrUvWJVFS+2xuGXJJwet6zELfIslWeU2+rqusIXw/FEw30/ulsg4bXTuQuEhFfs4PHpXM590vObE3eCz2ttYlXiI4qY1TfDBG1DAE7KRcQmH7Ptc7Lx+/";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0/9.0);
        }

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public boolean isArtifactDetected(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if ("duck".equalsIgnoreCase(recognition.getLabel()) || "cube".equalsIgnoreCase(recognition.getLabel()) || "ball".equalsIgnoreCase(recognition.getLabel()))
                    {
                        return true;
                    }
                    i++;
                }
            }
        }
        return false;
    }
}
