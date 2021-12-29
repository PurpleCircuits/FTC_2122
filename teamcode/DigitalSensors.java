package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DigitalSensors {
    private DigitalChannel slideSwitch1 = null;
    private DigitalChannel clawSwitch1 = null;
    private DigitalChannel clawSwitch2 = null;
    private DistanceSensor leftDistance = null;
    private DistanceSensor rightDistance = null;
    private DistanceSensor frontDistance = null;
    private ColorRangeSensor leftColor = null;
    private ColorRangeSensor rightColor = null;
    HardwareMap hwMap;

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        //slideSwitch1 = hwMap.get(DigitalChannel.class, "slide_switch_1");
        //slideSwitch1.setMode(DigitalChannel.Mode.INPUT);

        clawSwitch1 = hwMap.get(DigitalChannel.class, "claw_switch_1");
        clawSwitch1.setMode(DigitalChannel.Mode.INPUT);

        leftDistance = hwMap.get(DistanceSensor.class, "left_distance");
        //leftDistance = hwMap.get(DistanceSensor.class, "right_distance");
        //leftDistance = hwMap.get(DistanceSensor.class, "front_distance");
        leftColor = hwMap.get(ColorRangeSensor.class, "left_color");
        //rightColor = hwMap.get(ColorRangeSensor.class, "right_color");

        //clawSwitch2 = hwMap.get(DigitalChannel.class, "claw_switch_2");
        //clawSwitch2.setMode(DigitalChannel.Mode.INPUT);

    }
    /**
     * Checks to see if claw switch 2 is at limit.
     * if the digital channel returns true it's HIGH and the button is unpressed.
     * @return true if at bottom state
     */
    public boolean isSS1AtLimit(){
        return !slideSwitch1.getState();
    }
    /**
     * Checks to see if claw switch 1 is at limit
     * @return true if at bottom state
     */
    public boolean isCS1AtLimit(){
        return !clawSwitch1.getState();
    }
    /**
     * Checks to see if claw switch 2 is at limit
     * @return true if at top state
     */
    public boolean isCS2AtLimit(){
        return !clawSwitch2.getState();
    }
    public void colorSensorTest(Telemetry telemetry) {

        final float[] hsvValues = new float[3];
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = leftColor.getNormalizedColors();

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
