package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "LeftRed", group = "Linear Opmode")
public class LeftRed extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private DigitalSensors digitalSensors = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.7; // Start at halfway position
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor theSpinMotor = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private BNO055IMU imu = null;
    private DistanceSensor leftDistance = null;
    private DistanceSensor frontDistance = null;

    DuckPosition placement = new DuckPosition();
    private OpenCvCamera webcam = null;
    /**
     * Purple Circuits implementation of the runOpMode LeftRed.
     */
    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        waitForStart();
        String level = "left";
        switch (placement.pipeline.getAnalysis()) {
            case LEFT:
                level = "bottom";
                telemetry.addData("Left", level);
                telemetry.update();
                sleep(50);
                break;
            case CENTER:
                level = "center";
                telemetry.addData("Center", level);
                telemetry.update();
                sleep(50);
                break;
            case RIGHT:
                level = "top";
                telemetry.addData("Right", level);
                telemetry.update();
                sleep(50);
        }
        webcam.stopStreaming();
        theClawServo.setPosition(SERVO_MIN_POS);
        //forward towards tower
        moveBotDrive(45,1,0,0);
        //Set claw to position
        if ("bottom".equalsIgnoreCase(level)){
            runToClawPosition(600);
            sleep(250);
            turnRight(270,5);
            moveBotDrive(20,.5,0,0);
        } else if ("center".equalsIgnoreCase(level)){
            runToClawPosition(1000);
            sleep(250);
            turnRight(270,5);
            moveBotDrive(20,.5,0,0);
        } else {
            runToClawPosition(1600);
            sleep(250);
            turnRight(270,5);
            moveBotDrive(24,.5,0,0);
        }
        //turnRight(270,5);
        //frontToDistance(.25,8,1);
        //sleep(250);
        //open claw
        theClawServo.setPosition(SERVO_OPEN_POS);
        sleep(500);
        moveBotDrive(12,-1,0,0);
        sleep(100);
        turnLeft(90,5);
        runToClawPosition(500);
        leftToDistance(8,10);
        moveBotStrafe(8,0,-.5,0);
        sleep(100);
        moveBotDrive(12,-1,0,0);
        runToColor(-.5,6);
        //doubled due to half speed
        moveBotDrive(28,-.5,0,0);
        sleep(250);
        theSpinMotor.setPower(-.3);
        sleep(4000);
        theSpinMotor.setPower(0);
        sleep(250);
        runToColor(.5,10);
        moveBotDrive(8,1,0,0);
        leftToDistance(8,5);
        clawAction();
    }

    private void initHardware() {
        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theClawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theClawMotor.setDirection(DcMotor.Direction.FORWARD);
        theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");

        theSpinMotor = hardwareMap.get(DcMotor.class, "the_spin_motor");

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE);

        digitalSensors = new DigitalSensors();
        digitalSensors.init(hardwareMap);

        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        //purpleTensorFlow = new PurpleTensorFlow();
        //purpleTensorFlow.init(hardwareMap);
        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        placement.pipeline = new DuckPosition.SamplePipeline();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(placement.pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }
    private void moveBotStrafe(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineStrafeTime(inches);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    private void moveBotDrive(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineDriveTime(inches);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
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
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>3)
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
        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>3)
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
        double m = 21;
        return inches / m;
    }
    private double determineDriveTime(int inches){
        double m = 30;
        return inches / m;
    }
    private void clawAction(){
        theClawMotor.setTargetPosition(0);
        if(digitalSensors.isCS1AtLimit()){
            theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else{
            while(!digitalSensors.isCS1AtLimit()){
                theClawMotor.setPower(-.25);
            }
            theClawMotor.setPower(0);
        }
    }
    private void runToClawPosition(int tics){
        theClawMotor.setTargetPosition(tics);
        theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theClawMotor.setPower(.5);
        while (opModeIsActive() && theClawMotor.isBusy()){
            //potential telemetry here if needed
        }
        theClawMotor.setPower(0);
    }
    private void runToColor(double speed, int timeout){
        runtime.reset();
        while(opModeIsActive() && digitalSensors.getColors().red < .010 && runtime.seconds() < timeout)
        {
            trigmecanum.mecanumDrive(speed,0,0,false,false);
        }
        trigmecanum.mecanumDrive(0,0,0,false,false);
    }
    public void leftToDistance(int distance, int timeout){
        runtime.reset();
        while(opModeIsActive() && leftDistance.getDistance(CM) > distance && runtime.seconds() < timeout)
        {
            trigmecanum.mecanumDrive(0,1,0,false,false);
        }
        trigmecanum.mecanumDrive(0,0,0,false,false);
    }
    public void frontToDistance(double speed, int distance, int timeout){
        runtime.reset();
        while(opModeIsActive() && frontDistance.getDistance(CM) > distance && runtime.seconds() < timeout)
        {
            telemetry.addData("distance",frontDistance.getDistance(CM));
            telemetry.update();
            trigmecanum.mecanumDrive(speed,0,0,false,false);
        }
        trigmecanum.mecanumDrive(0,0,0,false,false);
    }
}

