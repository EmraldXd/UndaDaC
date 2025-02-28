package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

public class SpecimenSensorTest extends OpMode {
    ColorRangeSensor sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        if(sensor.getLightDetected() == 1) {
            //Insert servo stuffs here
        }
        telemetry.addData("Specimen in claw? ", sensor.getLightDetected() == 1 ? "Yes" : "No");
    }
}
