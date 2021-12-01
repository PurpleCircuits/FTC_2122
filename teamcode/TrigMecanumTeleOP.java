package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TrigMecanumTeleOP", group="Linear Opmode")
public class TrigMecanumTeleOP extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private Servo theClawServo = null;
    BNO055IMU imu;
    private DistanceSensor distanceSensor = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 0.7; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.4; // Start at halfway position
    private DcMotor theClawMotor = null;
    private DcMotor theSlideMotor = null;
    private DcMotor theSpinMotor = null;
    private DigitalSensors digitalSensors = null;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isArmMoving = false;
    private double armFinishTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            //distanceAction();
            clawAction();
            slideAction();
            spinAction();
            clawPosition();
            trigmecanum.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper, gamepad1.right_bumper);
            //telemetry.addData("tics",theClawMotor.getCurrentPosition());
            //telemetry.update();
        }
    }
    private void initHardware(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD);
        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawMotor.setDirection(DcMotor.Direction.FORWARD);
        theClawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        theSlideMotor = hardwareMap.get(DcMotor.class, "the_slide_motor");
        theSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        theSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        digitalSensors = new DigitalSensors();
        digitalSensors.init(hardwareMap);

        theSpinMotor = hardwareMap.get(DcMotor.class, "the_spin_motor");
        theSpinMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName", distanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
    }
    private void clawAction() {
        // close the claw
        if(gamepad2.dpad_down){
            theClawServo.setPosition(SERVO_MAX_POS);
        }
        if(gamepad2.left_bumper){
            theClawServo.setPosition(SERVO_OPEN_POS);
        }
        else if (gamepad2.right_bumper || 5 > distanceSensor.getDistance(DistanceUnit.CM)) {
            theClawServo.setPosition(SERVO_MIN_POS);
        }
        // Power for claw
        double power = -gamepad2.left_stick_y / 2;
        // Slow down the robot by factor of 2 when right bumper pressed
        if (gamepad2.right_bumper) {
            power = -gamepad2.left_stick_y;
        }
        if (digitalSensors.isCS1AtLimit() && 0 < gamepad2.left_stick_y) {
            theClawMotor.setPower(0);
            theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            theClawMotor.setPower(power);
        }
    }
    private void slideAction(){
        double power = gamepad2.right_stick_y;
        theSlideMotor.setPower(power);
    }
    private void spinAction(){
        double power = 0;
        double leftPower = gamepad2.left_trigger;
        double rightPower = gamepad2.right_trigger;
        if (leftPower > .1){
            if (leftPower > .5){
                power = .5;
            } else{
                power = leftPower;
            }
        } else {
            if (rightPower > .5){
                power = -.5;
            } else{
                power = -rightPower;
            }

        }
        theSpinMotor.setPower(power);
    }
    private void clawPosition(){
        if (isArmMoving){
            if (runtime.seconds() > armFinishTime){
                theClawMotor.setPower(0);
                isArmMoving = false;
                armFinishTime = 0;
            }
        }else {
            double time = 0;
            if (gamepad2.a) {
                time = .4;
            } else if (gamepad2.b) {
                time = .75;
            } else if (gamepad2.y) {
                time = 1;
            }
            if(time != 0){
                theClawMotor.setPower(.5);
                isArmMoving = true;
                runtime.reset();
                armFinishTime = time;
            }
        }
        /*if (gamepad2.x){
            if(digitalSensors.isCS1AtLimit()){
                theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else{
                while(!digitalSensors.isCS1AtLimit()){
                    theClawMotor.setPower(-.25);
                }
                theClawMotor.setPower(0);
            }
        } */
        //TODO set X for the capstone thing (need rev encoder things)
    }
}

