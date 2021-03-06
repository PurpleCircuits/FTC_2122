package org.firstinspires.ftc.teamcode.testrobotclasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Trigmecanum;

import java.util.List;

@Autonomous(name = "TestLeftBlue", group = "Linear Opmode")
@Disabled
public class TestLeftBlue extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.5;
    private static final double COUNTS_PER_MOTOR_REV = 1120 ;    // (40 GEARBOX) eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private BNO055IMU imu = null;
    private DistanceSensor distanceSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel digitalTouch = null;
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
    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        waitForStart();

        //TODO IMPORTANT! Stick 1X negative is LEFT
        //tensorflow find the cube
        String action;
        //sleep to find artifact
        sleep(500);
        if (isArtifactDetected()){
            action = "r";
            moveBot(8,-1,0,0);
        }
        else{
            moveBot(8,-1,0,0);
            //sleep to find artifact
            sleep(500);
            if (isArtifactDetected()){
                action = "c";
            } else {
                action = "l";
            }
        }
        telemetry.addData("artifact location", action);
        telemetry.update();
        sleep(10000);
        //if no cube reverse 8 inches
        //tensorflow find the cube
        //if no cube here we know its on the third square
        //strafe left to put things on the thing
        moveBotTime(determineStrafeTime(36),0,-1,0);
        if ("r".equalsIgnoreCase(action)){
            //set arm to position 1
        } else if ("c".equalsIgnoreCase(action)){
            //set arm to position 2
        } else {
            //set arm to position 3
        }
        moveBot(8,1,0,0);
        //TODO open claw
        moveBot(8,-1,0,0);
        //move back to where we started
        //turn to align with the opening
        turnLeft(90,5);
        moveBotTime(determineDriveTime(37),-1,0,0);
        //strafe left into the square
        moveBotTime(determineStrafeTime(36),0,-1,0);
        //go further into the loading dock
        moveBotTime(determineDriveTime(24), 1,0,0);
    }

    private void initHardware() {
        //theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        //theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "limit_sensor");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE);

        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }
    private void moveBot(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineStrafeTime(inches);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
        telemetry.addData("Drive", telemetryholder);
        telemetry.update();
    }
    private void moveBotTime(double timeoutS, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
        telemetry.addData("Drive", telemetryholder);
        telemetry.update();
    }
    public void turnLeft(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>2)
        {
            //Change the 10 on the line below to a variable
            scaledSpeed = degreesRemaining / (10 + degreesRemaining) * speed;
            if(scaledSpeed>1 || scaledSpeed<.5){scaledSpeed=.5;}//We have a minimum and maximum scaled speed

            trigmecanum.mecanumDrive(0,0, scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    //TODO see comments in turnLeft
    //ZYX, XYZ
    public void turnRight(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading = angles.firstAngle+turnAngle;
        if(targetHeading < -180) {targetHeading += 360;}
        if(targetHeading > 180){targetHeading -= 360;}
        double degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>2)
        {
            scaledSpeed=degreesRemaining/(10+degreesRemaining)*speed;
            if(scaledSpeed>1 || scaledSpeed<.5){scaledSpeed=.5;}//We have a minimum and maximum scaled speed

            trigmecanum.mecanumDrive(0,0, -scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    private double determineStrafeTime(int inches){
        double m = 11;
        return inches / m;
    }
    private double determineDriveTime(int inches){
        double m = 15;
        return inches / m;
    }

    //TODO Last years methods
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName", distanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
    }
    private void dropGoal() {
        if (!opModeIsActive()) {
            return;
        }
        if(isAtLimit()) {
            //Arm down until sensor
            theClawMotor.setPower(-.3);
            while(isAtLimit()){
            }
            theClawMotor.setPower(0);
        }
        theClawServo.setPosition(SERVO_MAX_POS);
        //Arm Up until sensor
        theClawMotor.setPower(.5);
        sleep(500);
        theClawMotor.setPower(0);
    }
    private boolean isAtLimit(){
        // send the info back to driver station using telemetry function.
        // if the digital channel returns true it's HIGH and the button is unpressed.
        return digitalTouch.getState();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
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
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    private boolean isArtifactDetected(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if ("cube".equalsIgnoreCase(recognition.getLabel()))
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