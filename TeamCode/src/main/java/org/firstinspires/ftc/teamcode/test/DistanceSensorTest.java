package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;


/**
 * This OpMode Exists so that we can test our Rev 2M Distance sensor. This allows us to know whether
 * or not we as programmers understand how the sensor works or not.
 */
@TeleOp
public class DistanceSensorTest extends OpMode {
    mecanumDrive mecanumDrive = new mecanumDrive();
    DistanceSensor distSense;
    @Override
    public void init() {
        distSense = hardwareMap.get(DistanceSensor.class,"distSense");
        mecanumDrive.init(this);
    }

    @Override
    public void loop() {
        if(distSense.getDistance(DistanceUnit.CM) < 5) {
            mecanumDrive.setPower(0, 1, 0);
        }
        telemetry.addData("Sensor Returning ", distSense.getDistance(DistanceUnit.CM) + " CM");
    }
}
