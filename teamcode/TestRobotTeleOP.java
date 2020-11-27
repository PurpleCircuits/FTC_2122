/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TestRobotTeleOP", group="Linear Opmode")
public class TestRobotTeleOP<pose> extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare our hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private DcMotor intake = null;
    private DistanceSensor bottomDistanceSensor = null;
    private Rev2mDistanceSensor sensorTimeOfFlight = null;
    NormalizedColorSensor colorSensor = null;
    private DistanceSensor topDistanceSensor = null;

    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_HALFWAY_POSITION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2;
    //private boolean isIntakeOn = true;

    private int a = 0;
    private int pose[] = new int[6];
    private int savedPosition = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
        //intake = hardwareMap.get(DcMotor.class, "toggle_intake");
        bottomDistanceSensor = hardwareMap.get(DistanceSensor.class, "bottomdistance");
        topDistanceSensor = hardwareMap.get(DistanceSensor.class, "topdistance");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        sensorTimeOfFlight = (Rev2mDistanceSensor) bottomDistanceSensor;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(2);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        theClawMotor.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            clawAction();
            driveAction();
            //intakeAction();
            //Below is used for outputting and testing distance sensor
            distanceAction();
            //The following tests color sensor output
            colorSensorTest();
            //TODO options below
            //Conveyer right stick #2
            //Launch#2 Y (add servo gate + conveyerAction)
            //Open Claw#2 B


            telemetry.update();
        }
    }
   /* private void intakeAction() {
        if (gamepad2.x) {
           isIntakeOn = !isIntakeOn;
        }
        if (isIntakeOn){
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }
    }
    */
    private void clawAction() {
        // close the claw
        if (gamepad2.a) {
            theClawServo.setPosition(SERVO_MIN_POS);
        }

        // open the claw
        if (gamepad2.b) {
            theClawServo.setPosition(SERVO_MAX_POS);
        }

        // Log the encoder value of the claw motor
        telemetry.addData("Claw Motor Encoder: ", "%d %d", theClawMotor.getCurrentPosition(), a);
        telemetry.addData("A value:", "%d", a);

        // This is a recreation of an exponential graph we decided to create
        // y = ax^2 with x being the joystick input and y being the motor power
        /*
        float x = gamepad2.left_stick_y;
        telemetry.addData("Claw Joystick: ", "%.2f", x);
        if ( x > 0) {
            double power = 0.90 * x * x;
            telemetry.addData("Claw Power (Positive): ", "%.2f", power);
            theClawMotor.setPower(power);
        } else {
            double power = -0.90 * x * x;
            telemetry.addData("Claw Power (Negative): ", "%.2f", power);
            theClawMotor.setPower(power);
        }
         */

        // Linear speed
        double power = gamepad2.left_stick_y;
        // Slow down the robot by factor 5 or 2 when right bumper pressed
        if (!gamepad2.right_bumper) {
            power = power / 5;
        } else {
            power = power / 2;
        }
        theClawMotor.setPower(power);

        // Save the position of the encoder when bumper is pressed
        if (gamepad2.left_bumper) {
            savedPosition = theClawMotor.getTargetPosition();
        }

        // Loop to get to position
        /*
        if (gamepad2.left_bumper){
            if (a<5){
                ++a;qqqmnaqaa[
            }else
               a = 0;
        }
        theClawMotor.setTargetPosition(pose[a]);
        theClawMotor.setPower(.5);
        theClawMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        */
        //sleep so bot does not over correct and to make sure one button press is one increment in 'a' value
        sleep(100);

    }

    private void driveAction() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0;
        double rightPower = 0;

        if (gamepad1.left_bumper) {
            // Set to tank mode
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        } else {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
        }
        // Slow down the robot by factor 5
        if (!gamepad1.right_bumper) {
            leftPower = leftPower / 3;
            rightPower = rightPower / 3;
        }
        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        // Telemetry wont be updated until update is called
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName", bottomDistanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", bottomDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("deviceName", topDistanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", topDistanceSensor.getDistance(DistanceUnit.CM)));

    }
    private void colorSensorTest() {

        final float[] hsvValues = new float[3];
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
    }
}