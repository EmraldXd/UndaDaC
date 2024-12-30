package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

public class roller {
    static final DecimalFormat df = new DecimalFormat("0.00");
    Telemetry telemetry;
    private CRServo rightWheel;
    private CRServo leftWheel;
    private Servo joint;
    private ColorRangeSensor sensor;
    private boolean up;
    private double DELAY = 0.75;
    private boolean spin;
    private boolean spinOut;
    private boolean lastPress; //This will make sure the intake stops or starts only if the button was released first
    private static final ElapsedTime buttonDelay = new ElapsedTime();

    public void init(@NonNull OpMode opmode){
        HardwareMap hardwareMap = opmode.hardwareMap;
        telemetry = opmode.telemetry;
        rightWheel = hardwareMap.get(CRServo.class, "rightW");
        leftWheel = hardwareMap.get(CRServo.class, "leftW");
        joint = hardwareMap.get(Servo.class, "joint");
        sensor = hardwareMap.get(ColorRangeSensor.class, "Color Sensor");
        joint.setPosition(0.01);
        joint.setPosition(0);
        up = true;
        spin = false;
        spinOut = false;
    }

    public void moveRoller(boolean input) {
        if(input && buttonDelay.time() >= DELAY && up) {
            joint.setPosition(0.2);
            up = !up;
            buttonDelay.reset();
        } else if (input && buttonDelay.time() >= DELAY && !up) {
            joint.setPosition(0);
            up = !up;
            buttonDelay.reset();
        }
    }

    public void intake(boolean intake, boolean outtake){
        if(intake && !clawLoaded() && !spinOut) {
            rightWheel.setPower(1);
            leftWheel.setPower(-1);
            spin = true;
        } else if(intake && !spin) {
            rightWheel.setPower(-1);
            leftWheel.setPower(1);
            spinOut = true;
        } else {
            rightWheel.setPower(0);
            leftWheel.setPower(0);
        }

        if(!intake) {
            spin = false;
            spinOut = false;
        }
    }

    public void lift(boolean lift) {
        if(!up && lift) {
            moveRoller(true);
        }
    }

    public boolean clawLoaded() {
        if(sensor.getLightDetected() == 1) {
            return true;
        } else {
            return false;
        }
    }

    public void telemetry() {
        telemetry.addData("Right Wheel Power: ", rightWheel.getPower());
        telemetry.addData("Left Wheel Power: ", rightWheel.getPower());
        telemetry.addData("Current Joint position: ", joint.getPosition());
        telemetry.addData("Claw loaded? ", clawLoaded());
        telemetry.addData("Current Color: ", sensor.getLightDetected());
    }
}
