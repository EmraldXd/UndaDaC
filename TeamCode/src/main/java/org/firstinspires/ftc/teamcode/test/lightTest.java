package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
