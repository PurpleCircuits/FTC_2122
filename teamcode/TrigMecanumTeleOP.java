package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TrigMecanumTeleOP", group="Linear Opmode")
public class TrigMecanumTeleOP extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private Servo theClawServo = null;
    BNO055IMU imu;
    private DistanceSensor distanceSensor = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.6; // Start at halfway position
    private DcMotor theClawMotor = null;
    private DcMotor theSlideMotor = null;
    private DcMotor theSpinMotor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            distanceAction();
            clawAction();
            slideAction();
            trigmecanum.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x, gamepad1.a, gamepad1.y);
            telemetry.update();
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


        //theSpinMotor = hardwareMap.get(DcMotor.class, "the_spin_motor");
        //theSpinMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName", distanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
    }
    private void clawAction() {
        // close the claw
        //if (gamepad2.b) {
        //    theClawServo.setPosition(SERVO_MIN_POS);
        //}

        // open the claw
        if (gamepad2.a || 2 > distanceSensor.getDistance(DistanceUnit.CM)) {
            theClawServo.setPosition(SERVO_MIN_POS);
        }
        else if(gamepad2.b){
            theClawServo.setPosition(SERVO_OPEN_POS);
        }
        // Linear speed
        double power = -gamepad2.left_stick_y;
        // Slow down the robot by factor 5 or 2 when right bumper pressed
        if (gamepad2.right_bumper) {
            power = power / 2;
        }
        theClawMotor.setPower(power);
    }
    private void slideAction(){
        double power = gamepad2.right_stick_y;
        theSlideMotor.setPower(power);
    }
    private void spinAction(){
        double power = 0;
        if (gamepad2.x){
            power = 0.5;
        }
        theSpinMotor.setPower(power);
    }
}

