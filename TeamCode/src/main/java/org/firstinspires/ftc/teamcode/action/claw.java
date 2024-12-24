package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class claw {
    static final DecimalFormat df = new DecimalFormat("0.00");
    Telemetry telemetry;
    private double DELAY = 0.75;
    private static final ElapsedTime anglerDelay = new ElapsedTime();
    private static final ElapsedTime delayToggle = new ElapsedTime();
    private static final ElapsedTime delayClaw = new ElapsedTime();
    private static final ElapsedTime jointDelay = new ElapsedTime();
    private Servo claw;
    private Servo clawJoint; //used to be ClawJointA
    private Servo clawJointB;
    private Servo clawAngle;
    private boolean isClosed;
    private boolean isTurned;
    private boolean clawToggle;
    private boolean up;
    public void init(@NonNull OpMode opmode){
        HardwareMap hardwareMap = opmode.hardwareMap;
        telemetry = opmode.telemetry;
        //initialize servo motors
        claw = hardwareMap.get(Servo.class, "Claw");
        clawJoint = hardwareMap.get(Servo.class, "Joint"); //used to be known as Claw Joint A
        //clawJointB = hardwareMap.get(Servo.class, "Joint B");
        clawAngle = hardwareMap.get(Servo.class, "Angler");
        isClosed = false;
        isTurned = false;
        clawToggle = false;
        up = true;
        //Turn Servos on
        clawAngle.setPosition(0);
        clawJoint.setPosition(0);
        claw.setPosition(.45);
    }

    public void startTime() {
        jointDelay.reset();
        delayClaw.reset();
        delayToggle.reset();
        anglerDelay.reset();
    }

    public void moveClaw(boolean a) {
        if (a && jointDelay.time() > DELAY) {
            clawJoint.setPosition(up ? .15 : 0 );
            up = !up;
            jointDelay.reset();
        }
    }

    public void useClaw(boolean isPressed) {
        if (isPressed && delayClaw.time() > DELAY && !isClosed) {
            claw.setPosition(0.45);
            isClosed = true;
            delayClaw.reset();
        } else if (isPressed && delayClaw.time() > DELAY && isClosed) {
            claw.setPosition(0.75);
            isClosed = false;
            delayClaw.reset();
        }
    }

    public void rotateClaw(boolean isLeftPressed, boolean isRightPressed) {
        if(isLeftPressed && anglerDelay.time() > DELAY) {
            clawAngle.setPosition(0.3);
            isTurned = true;
            anglerDelay.reset();
        } else if (isRightPressed && anglerDelay.time() > DELAY){
            clawAngle.setPosition(0);
            isTurned = false;
            anglerDelay.reset();
        }
    }

    public void lift(boolean lift) {
        if(!up && lift) {
            moveClaw(true);
        }
    }

    public void showTelemetry() {
        telemetry.addData("Elapsed time (claw): ", df.format(delayClaw.time()));
        telemetry.addData("Elapsed time (joint): ", df.format(jointDelay.time()));
        telemetry.addData("Elapsed time (angler): ", df.format(anglerDelay.time()));
        telemetry.addData("Elapsed time (toggle): ", df.format(delayToggle.time()));
        telemetry.addData("Claw Position: ", df.format(claw.getPosition()));
        telemetry.addData("Joint A position: ", df.format(clawJoint.getPosition()));
        telemetry.addData("Angler Position: ", df.format(clawAngle.getPosition()));
    }
}
