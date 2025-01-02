package org.firstinspires.ftc.teamcode.customAction;

import android.app.Notification;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class intakeRR {
    CRServo rightServo;
    CRServo leftServo;
    Servo joint;
    ColorRangeSensor sensor;
    private static final ElapsedTime stopDelay = new ElapsedTime();
    public intakeRR(HardwareMap hardwareMap) {
        rightServo = hardwareMap.get(CRServo.class, "rightW");
        leftServo = hardwareMap.get(CRServo.class, "leftW");
        joint = hardwareMap.get(Servo .class, "joint");
        sensor = hardwareMap.get(ColorRangeSensor .class, "Color Sensor");
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class pickup implements Action {

        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!init) {
                init = !init;
                rightServo.setPower(1);
                leftServo.setPower(1);
            }

            if (sensor.getLightDetected() == 1) {
                rightServo.setPower(0);
                leftServo.setPower(0);
            }
            return !(sensor.getLightDetected() == 1);
        }
    }

    public class putDown implements Action {

        private boolean init = false;
        private boolean finished = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!init) {
                init = !init;
                rightServo.setPower(-1);
                leftServo.setPower(-1);
            }

            if(sensor.getLightDetected() < 1 && stopDelay.time() > 1.00) {
                stopDelay.reset();
            } else if(sensor.getLightDetected() < 1 && stopDelay.time() > 0.5) {
                rightServo.setPower(0);
                leftServo.setPower(0);
                finished = true;
            }
            return !finished;
        }
    }

    public class moveJoint implements Action {
        private boolean moved = false;
        private double pos;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(joint.getPosition() != 0 && !moved) {
                joint.setPosition(0);
                moved = true;
                pos = 0;
            } else if (!moved){
                joint.setPosition(0.2);
                moved = true;
                pos = 0.2;
            }
            return !(joint.getPosition() == pos);
        }
    }

    public Action movejoint() {
        return new moveJoint();
    }

    public Action take() {
        return new pickup();
    }

    public Action place() {
        return new putDown();
    }
}
