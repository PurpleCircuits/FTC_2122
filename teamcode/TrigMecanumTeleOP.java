package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TrigMecanumTeleOP", group="Linear Opmode")
public class TrigMecanumTeleOP extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private Servo theClawServo = null;
    BNO055IMU imu;
    private ColorRangeSensor clawDistance = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1; // Maximum rotational position
    private static final double SERVO_OPEN_POS = 0.7; // Start at halfway position
    private DcMotor theClawMotor = null;
    private DcMotor theSlideMotor = null;
    private DcMotor theSpinMotor = null;
    private DigitalSensors digitalSensors = null;
    private boolean isArmMoving = false;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            clawAction();
            slideAction();
            spinAction();
            clawPosition();
            trigmecanum.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper, gamepad1.right_bumper);
            //Color Sensor output
            //digitalSensors.colorSensorTest(telemetry);
            //Claw motor tics output
            //telemetry.addData("tics",theClawMotor.getCurrentPosition());
            //Claw Distance sensor output
            telemetry.addData("Claw Distance:",clawDistance.getDistance(CM));
            telemetry.update();
        }
    }
    private void initHardware(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        clawDistance = hardwareMap.get(ColorRangeSensor.class, "distance");
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD);

        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theClawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theClawMotor.setDirection(DcMotor.Direction.FORWARD);

        theSlideMotor = hardwareMap.get(DcMotor.class, "the_slide_motor");
        theSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        theSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        digitalSensors = new DigitalSensors();
        digitalSensors.init(hardwareMap);

        theSpinMotor = hardwareMap.get(DcMotor.class, "the_spin_motor");
        theSpinMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    private void clawAction() {
        // close the claw
        if(gamepad2.dpad_down){
            theClawServo.setPosition(SERVO_MAX_POS);
        }
        else if(gamepad2.left_bumper){
            theClawServo.setPosition(SERVO_OPEN_POS);
        }
        else if (gamepad2.right_bumper || 5 > clawDistance.getDistance(DistanceUnit.CM)) {
            theClawServo.setPosition(SERVO_MIN_POS);
        }
    }
    private void slideAction(){
        double power = gamepad2.left_stick_y;
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
    private void clawPosition()
    {
        if (isArmMoving)
        {
            if (!opModeIsActive() || !theClawMotor.isBusy()){
                theClawMotor.setPower(0);
                isArmMoving = false;
            }
        } else {
            boolean buttonpushed = false;
            if (gamepad2.a) {
                buttonpushed = true;
                theClawMotor.setTargetPosition(500);
            } else if (gamepad2.b) {
                buttonpushed = true;
                theClawMotor.setTargetPosition(1000);
            } else if (gamepad2.y) {
                buttonpushed = true;
                theClawMotor.setTargetPosition(0);
            }

            if (buttonpushed){
                theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                theClawMotor.setPower(.5);
                isArmMoving = true;
            } else {
                // Power for claw
                double power = -gamepad2.right_stick_y / 2;
                telemetry.addData("ClawMotorPower",power);

                if (digitalSensors.isCS1AtLimit() && 0 < gamepad2.right_stick_y) {
                    theClawMotor.setPower(0);
                    theClawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    theClawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("IN MOVE ClawMotorPower",power);
                    theClawMotor.setPower(power);
                }
            }
        }
        telemetry.addData("isarmmoving", isArmMoving);
    }
}