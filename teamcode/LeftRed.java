package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "LeftRed", group = "Linear Opmode")
public class LeftRed extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private DigitalSensors digitalSensors = null;
    private PurpleTensorFlow purpleTensorFlow = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.6; // Start at halfway position
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
    private DcMotor theSpinMotor = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private BNO055IMU imu = null;
    private DistanceSensor topDistanceSensor = null;
    private DistanceSensor bottomDistanceSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel slideSwitch1 = null;
    private DigitalChannel clawSwitch1 = null;
    private DigitalChannel clawSwitch2 = null;
    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        waitForStart();
        //TODO load the box off the ground a little bit
        theClawServo.setPosition(SERVO_MIN_POS);
        //tensorflow find the cube
        String action;
        //sleep to give time to find artifact
        sleep(4000);
        if (purpleTensorFlow.isArtifactDetected()){
            action = "r";
            moveBotStrafe(8,0,-1,0);
        }
        else{
            moveBotStrafe(8,0,-1,0);
            //sleep to find artifact
            sleep(4000);
            if (purpleTensorFlow.isArtifactDetected()){
                action = "c";
            } else {
                action = "l";
            }
        }
        telemetry.addData("artifact location", action);
        telemetry.update();
        //if no cube reverse 8 inches
        //tensorflow find the cube
        //if no cube here we know its on the third square
        //forward towards tower
        moveBotDrive(36,1,0,0);
        //turn to fully align with goal
        turnRight(270,10);
        if ("l".equalsIgnoreCase(action)){
            moveClaw(65);
        } else if ("c".equalsIgnoreCase(action)){
            moveClaw(130);
        } else {
            moveClaw(180);
        }
        moveBotDrive(8,1,0,0);
        //open claw
        theClawServo.setPosition(SERVO_OPEN_POS);
        //go back
        moveBotDrive(8,-1,0,0);
        //turn and align with carousel
        turnLeft(45,10);
        //reverse to carousel
        moveBotDrive(50,-1,0,0);
        //spin carousel
        theSpinMotor.setPower(.5);
        //TODO change this to a while loop timeout
        sleep(4000);
        //move away from carousel
        moveBotDrive(5,1,0,0);
        //turn to align straight
        turnRight(315,0);
        //strafe to align with blue dock
        moveBotStrafe(20,0,-1,0);
        //reverse to wall
        moveBotDrive(18,-1,0,0);
    }

    private void initHardware() {
        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");

        theSpinMotor = hardwareMap.get(DcMotor.class, "the_spin_motor");

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE);

        digitalSensors = new DigitalSensors();
        digitalSensors.init(hardwareMap);

        purpleTensorFlow = new PurpleTensorFlow();
        purpleTensorFlow.init(hardwareMap);
        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        telemetry.addData("Drive", telemetryholder);
        telemetry.update();
    }
    private void moveBotDrive(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineDriveTime(inches);
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
        double m = 21;
        return inches / m;
    }
    private double determineDriveTime(int inches){
        double m = 30;
        return inches / m;
    }
    private void clawAction(){
        if(digitalSensors.isCS1AtLimit()){
            theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else{
            while(!digitalSensors.isCS1AtLimit()){
                theClawMotor.setPower(-.25);
            }
            theClawMotor.setPower(0);
        }
    }
    private void moveClaw(int tics){
        theClawMotor.setTargetPosition(tics);
        theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theClawMotor.setPower(.5);
        while (opModeIsActive() && theClawMotor.isBusy());
        theClawMotor.setPower(0);
        theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
