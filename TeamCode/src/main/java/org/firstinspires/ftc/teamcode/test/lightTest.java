package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This OpMode is here so that we can learn how to adjust the power of the robot light. We ended up
 * not using this in the final build, but it was used to figure out how things work.
 */
@Disabled
@TeleOp
public class lightTest extends OpMode {
    Servo light;
    boolean brighten;

    @Override
    public void init() {
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(1);
        brighten = true;
    }

    @Override
    public void loop() {
        light.setPosition(light.getPosition() + (brighten ? 0.1:-0.1));
        if(light.getPosition() == 1 || light.getPosition() == 0) {
            brighten = !brighten;
        }
    }
}
