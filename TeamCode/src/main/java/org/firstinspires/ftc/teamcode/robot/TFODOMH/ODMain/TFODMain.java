package org.firstinspires.ftc.teamcode.robot.TFODOMH.ODMain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Matrix4fBuilder;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector3f;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Config
@Autonomous(name = "TFOD_MAIN", group = "Test")
public class TFODMain extends OpMode {

    //illustration key
    private static final String VUFORIA_KEY = "AV9rwXT/////AAABma+8TAirNkVYosxu9qv0Uz051FVEjKU+nkH+MaIvGuHMijrdgoZYBZwCW2aG8P3+eZecZZPq9UKsZiTHAg73h09NT48122Ui10c8DsPe0Tx5Af6VaBklR898w8xCTdOUa7AlBEOa4KfWX6zDngegeZT5hBLfJKE1tiDmYhJezVDlITIh7SHBv0xBvoQuXhemlzL/OmjrnLuWoKVVW0kLanImI7yra+L8eOCLLp1BBD/Iaq2irZCdvgziZPnMLeTUEO9XUbuW8txq9i51anvlwY8yvMXLvIenNC1xg4KFhMmFzZ8xnpx4nWZZtyRBxaDU99aXm7cQgkVP0VD/eBIDYN4AcB0/Pa7V376m6tRJ5UZh";

    //load assets and initialize labelling
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

    private static final String CAMERA_NAME = "MainCam";

    //var for isBusy
    private boolean isBusy = false;

    //Enable debug?
    private boolean debug = true;

    //image properties of the camera feed
    private int imgWidth = 0, imgHeight = 0;

    //object recognition variables
    private int objsListSize = 0;
    private HashMap<String, Boolean> objHMDetected;
    private ArrayList<Float> bbLeft = null, bbRight = null, bbTop = null, bbBottom = null; //coordinates for debugging
    private ArrayList<Vector2f> bbTopLeft = null, bbBottomRight = null; //in the NDC standard for graphics
    private Vector3f local_pos = new Vector3f();

    //object to describe the camera frustum
    private FrustumInterpolator l270;

    public TFODMain(){init();}

    @Override
    public void init() {
        isBusy = true;

        initVuforia();
        initTensorFlow();
        initObjectRecognitionVariables();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        l270 = FrustumInterpolator.Logitech_C270; //get preset for the Logitech C270 webcam that includes the FOV for both Vertical & Horizontal

        l270.setCamPos(new Vector3f(3.8f, 7.4f, 6.8f)); //describe the position of the camera ***lens***
        l270.setCamRot(Matrix4fBuilder.buildGenRot(325, 0, 180)); //describe the rotation of the camera ***lens***
        l270.setupFrustum();

        isBusy = false;
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        scan();
        Vector2f bb = calculateBBVector();
        if (bb.getY() == -2) {
            local_pos = l270.convertIMGCoord(bb);
        }

        if (isBusy == false && debug == true) {
            telemetry.addData("Object List Size: ", objsListSize);
            telemetry.addData("Objects Detected: ", objHMDetected.toString());
            telemetry.addData("BBLeft: ", bbLeft);
            telemetry.addData("BBTop: ", bbTop);
            telemetry.addData("BBRight: ", bbRight);
            telemetry.addData("BBBottom: ", bbBottom);
            telemetry.addData("BBTOPLEFT: ", bbTopLeft);
            telemetry.addData("BBBOTRIGHT: ", bbBottomRight);
            telemetry.addData("Null? [bbTopLeft, bbBottomRight]: ", "[" + (bbTopLeft == null) + ", " + (bbBottomRight == null) + "]");
            telemetry.addData("BBClosest: ", bb);
            telemetry.addData("BB to Local: ", local_pos);
            telemetry.addData("Cam HFOV: ", l270.gethFOV());
            telemetry.addData("Cam VFOV: ", l270.getvFOV());
            telemetry.addData("Bottom: ", l270.getFplane_bottom());
            telemetry.addData("Right:", l270.getFplane_right());
            telemetry.addData("Center: ", l270.getFplane_center());
            telemetry.addData("Cam Rot Matrix: ", l270.getCamRot());
        }
    }

    @Override
    public void stop(){
        if (tfod != null){
            tfod.shutdown();
        }
    }

    /**
     * PUBLIC CLASS METHODS UNDER HERE
     */

    /**
     * This method rescans the image for objects when called for flexibility if you aren't happy with loop()
     */
    public void scan(){
        isBusy = true; //set the state of the scan to busy

        if (tfod != null) {

            //clear the lists so it won't overflow memory after a while
            bbLeft.clear();
            bbTop.clear();
            bbRight.clear();
            bbBottom.clear();
            bbTopLeft.clear();
            bbBottomRight.clear();

            //get recognitions
            List<Recognition> tfodRecognitions = tfod.getRecognitions();
            if (tfodRecognitions != null){
                objsListSize = tfodRecognitions.size();

                objHMDetected.put(LABELS[0], false); //Ball
                objHMDetected.put(LABELS[1], false); //Cube
                objHMDetected.put(LABELS[2], false); //Duck
                objHMDetected.put(LABELS[3], false); //Marker

                for (Recognition recognition : tfodRecognitions){
                    if (recognition.getLabel() != null){
                        String bbLabel = recognition.getLabel().toUpperCase();

                        //adds the bounding box coordinates to their appropriate lists
                        bbLeft.add(recognition.getLeft());
                        bbTop.add(recognition.getTop());
                        bbRight.add(recognition.getRight());
                        bbBottom.add(recognition.getBottom());

                        Vector2f normalizedTL = new Vector2f(recognition.getLeft() / imgWidth * 2 - 1, recognition.getTop() / imgHeight * 2 - 1);
                        Vector2f normalizedBR = new Vector2f(recognition.getRight() / imgWidth * 2 - 1, recognition.getBottom() / imgHeight * 2 - 1);


                        //identify the recognized object, sets the detection status for all of them
                        switch (bbLabel) {
                            case "BALL":
                                objHMDetected.put(LABELS[0], true); //Ball
                                bbTopLeft.add(normalizedTL);
                                bbBottomRight.add(normalizedBR);
                                break;
                            case "CUBE":
                                objHMDetected.put(LABELS[1], true); //Cube
                                bbTopLeft.add(normalizedTL);
                                bbBottomRight.add(normalizedBR);
                                break;
                            case "DUCK":
                                objHMDetected.put(LABELS[2], true); //Duck
                                bbTopLeft.add(normalizedTL);
                                bbBottomRight.add(normalizedBR);
                                break;
                            case "MARKER":
                                objHMDetected.put(LABELS[3], true); //Market
                                break;
                        }
                    }
                }
            }

        }

        isBusy = false; //set the state of the scan to be not busy since it's finished
    }

    /**
     * Calculates the center of the bounding boxes and find which are the closest to the center of screen space
     * @return closest Vector2f to the center of the image
     */
    public Vector2f calculateBBVector(){
        isBusy = true;
        //check for null pointers because those are always sneaky
        if ((bbTopLeft.size() != 0 && bbTopLeft != null) && (bbBottomRight.size() != 0 && bbBottomRight != null)){

            int loop_len = Math.min(bbTopLeft.size(), bbBottomRight.size()); //you can't guarantee the sizes are going to be the same, so as a precaution, I threw this here to prevent IndexOutOfArrayBounds exceptions
            float previous_len = 2; //length from the center, plus we are working in NDC space so the bounds for both x and y are between -1 & 1
            Vector2f temp_tl, temp_br, center, closest = new Vector2f(0, 0); //allocate mem to these objects so I don't have to repeat the creation of an object in the for loop; also instantiate closest just in case somehow one of the elements is null

            for (int i = 0; i < loop_len; i++){
                //store the Vector2f gotten from the arraylist to avoid accidental mutation
                temp_tl = bbTopLeft.get(i);
                temp_br = bbBottomRight.get(i);

                //check for null element, but this is probably redundant
                if (temp_tl != null && temp_br != null) {
                    //find the center of the bounding box
                    center = Vector2f.add(temp_tl, temp_br);
                    center.div(2);

                    //if center length is amazing and closest to the center, we overwrite closest and previous_len with center and center.length() respectively
                    if (center.length() < previous_len) {
                        previous_len = center.length();
                        closest = center;
                    }
                }
            }

            isBusy = false;
            return closest; //returns closest Vector2f to be processed as a Vector3f later to find depth then local space coordinates
        }

        isBusy = false;
        return new Vector2f(0, -2); //if we can't find the nearest, return a vector out of bounds
    }


    /**
     * INITIALIZERS UNDER HERE
     */

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
        parameters.cameraName = hardwareMap.get(WebcamName.class, TFODMain.CAMERA_NAME);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = null;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackable is not necessary for the TensorFlow Object Detection engine.
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

    /**
     * Initialize a few important variables, such as: bounding box position, image properties, obj recognition lists, etc.
     */
    public void initObjectRecognitionVariables(){
        //get camera spec (Width and Height of the output camera bitmap)
        Size s = vuforia.getCameraCalibration().getSize();
        imgWidth = s.getWidth();
        imgHeight = s.getHeight();

        //See how many objects we detect
        objsListSize = 0;

        //Coordinate Lists
        bbLeft = new ArrayList<>();
        bbTop = new ArrayList<>();
        bbRight = new ArrayList<>();
        bbBottom = new ArrayList<>();

        //initialize vector list
        bbTopLeft = new ArrayList<>();
        bbBottomRight = new ArrayList<>();

        //Detection List for Debugging
        objHMDetected = new HashMap<>();
        objHMDetected.put(LABELS[0], false); //Ball
        objHMDetected.put(LABELS[1], false); //Cube
        objHMDetected.put(LABELS[2], false); //Duck
        objHMDetected.put(LABELS[3], false); //Marker
    }

    /**
     * GETTERS AND SETTERS HERE
     */

    /**
     * Checks if this class is busy with an action
     * @return boolean isBusy
     */
    public boolean isBusy(){
        return isBusy;
    }
}
