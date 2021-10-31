package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DigitalSensors {
    private DigitalChannel slideSwitch1 = null;
    private DigitalChannel clawSwitch1 = null;
    private DigitalChannel clawSwitch2 = null;
    HardwareMap hwMap;

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        slideSwitch1 = hwMap.get(DigitalChannel.class, "slide_switch_1");
        slideSwitch1.setMode(DigitalChannel.Mode.INPUT);

        clawSwitch1 = hwMap.get(DigitalChannel.class, "claw_switch_1");
        clawSwitch1.setMode(DigitalChannel.Mode.INPUT);

        clawSwitch2 = hwMap.get(DigitalChannel.class, "claw_switch_2");
        clawSwitch2.setMode(DigitalChannel.Mode.INPUT);

    }
    /**
     * Checks to see if claw switch 2 is at limit.
     * if the digital channel returns true it's HIGH and the button is unpressed.
     * @return false if at bottom state
     */
    public boolean isSS1AtLimit(){
        return slideSwitch1.getState();
    }
    /**
     * Checks to see if claw switch 1 is at limit
     * @return false if at bottom state
     */
    public boolean isCS1AtLimit(){
        return clawSwitch1.getState();
    }
    /**
     * Checks to see if claw switch 2 is at limit
     * @return false if at top state
     */
    public boolean isCS2AtLimit(){
        return clawSwitch2.getState();
    }


}
