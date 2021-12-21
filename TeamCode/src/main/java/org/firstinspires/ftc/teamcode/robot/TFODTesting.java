package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.function.BiConsumer;

@Config
@Autonomous(name = "TFOD_TEST", group = "Test")
public class TFODTesting extends OpMode {

    //illustration key
    private static final String VUFORIA_KEY = "AV9rwXT/////AAABma+8TAirNkVYosxu9qv0Uz051FVEjKU+nkH+MaIvGuHMijrdgoZYBZwCW2aG8P3+eZecZZPq9UKsZiTHAg73h09NT48122Ui10c8DsPe0Tx5Af6VaBklR898w8xCTdOUa7AlBEOa4KfWX6zDngegeZT5hBLfJKE1tiDmYhJezVDlITIh7SHBv0xBvoQuXhemlzL/OmjrnLuWoKVVW0kLanImI7yra+L8eOCLLp1BBD/Iaq2irZCdvgziZPnMLeTUEO9XUbuW8txq9i51anvlwY8yvMXLvIenNC1xg4KFhMmFzZ8xnpx4nWZZtyRBxaDU99aXm7cQgkVP0VD/eBIDYN4AcB0/Pa7V376m6tRJ5UZh";

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //object recognition variables
    private int objsListSize = 0;
    private HashMap<String, Boolean> objHMDetected;
    private ArrayList<Float> bbLeft = null, bbRight = null, bbTop = null, bbBottom = null; //coordinates

    @Override
    public void init() {
        initVuforia();
        initTensorFlow();
        initObjectRecognitionVariables();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.1, 16.0/9.0);
        }
    }



    @Override
    public void loop() {
        //clear the lists so it won't overflow memory after a while
        bbLeft.clear();
        bbTop.clear();
        bbRight.clear();
        bbBottom.clear();

        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null){
                objsListSize = updatedRecognitions.size();

                objHMDetected.put(LABELS[0], false); //Ball
                objHMDetected.put(LABELS[1], false); //Cube
                objHMDetected.put(LABELS[2], false); //Duck
                objHMDetected.put(LABELS[3], false); //Marker

                for (Recognition recognition : updatedRecognitions){
                    if (recognition.getLabel() != null){
                        String bbLabel = recognition.getLabel().toUpperCase();

                        //adds the bounding box coordinates to their appropriate lists
                        bbLeft.add(recognition.getLeft());
                        bbTop.add(recognition.getTop());
                        bbRight.add(recognition.getRight());
                        bbBottom.add(recognition.getBottom());

                        switch (bbLabel) {
                            case "BALL":
                                objHMDetected.put(LABELS[0], true); //Ball
                                break;
                            case "CUBE":
                                objHMDetected.put(LABELS[1], true); //Ball
                                break;
                            case "DUCK":
                                objHMDetected.put(LABELS[2], true); //Ball
                                break;
                            case "MARKER":
                                objHMDetected.put(LABELS[3], true); //Ball
                                break;
                        }
                    }
                }
            }

        }

        telemetry.addData("Object List Size: ", objsListSize);
        telemetry.addData("Objects Detected: ", objHMDetected.toString());
        telemetry.addData("BBLeft: ", bbLeft);
        telemetry.addData("BBTop: ", bbTop);
        telemetry.addData("BBRight: ", bbRight);
        telemetry.addData("BBBottom: ", bbBottom);
    }

    @Override
    public void stop(){
        if (tfod != null){
            tfod.shutdown();
        }
    }

    /**
     * Initializes Vuforia for us to use
     * <p>This method gives the localizer the VUFORIA_KEY;
     */
    public void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "MainCam");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true;
        parameters.cameraMonitorFeedback = null;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initializes TF for us to use
     * <p>This method inits: minResultConfidence; ModelTensorFlow2; inputSize;
     */
    public void initTensorFlow(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.69f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.useObjectTracker = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void initObjectRecognitionVariables(){
        objsListSize = 0;

        //Coordinate Lists
        bbLeft = new ArrayList<>();
        bbTop = new ArrayList<>();
        bbRight = new ArrayList<>();
        bbBottom = new ArrayList<>();

        objHMDetected = new HashMap<>();
        objHMDetected.put(LABELS[0], false); //Ball
        objHMDetected.put(LABELS[1], false); //Cube
        objHMDetected.put(LABELS[2], false); //Duck
        objHMDetected.put(LABELS[3], false); //Marker
    }
}
