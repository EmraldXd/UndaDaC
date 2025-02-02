package org.firstinspires.ftc.teamcode.customAction;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawRR {
    Servo joint;
    Servo claw;
    Servo angler;
    double placeholder;
    public clawRR(HardwareMap hardwareMap) {
        joint = hardwareMap.get(Servo.class, "Joint");
        claw = hardwareMap.get(Servo.class, "Claw");
        angler = hardwareMap.get(Servo.class, "Angler");
    }

    public class initialize implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0);
            joint.setPosition(0);
            angler.setPosition(0);
            if(claw.getPosition() == 0 && angler.getPosition() == 0 && joint.getPosition() == 0){
                return false;
            } else {
                return true;
            }
        }
    }

    public class grab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0);
            return false;
        }
    }

    public class prep implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0.35);
            return false;
        }
    }

    public class moveArm implements Action {

        private boolean finished;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(joint.getPosition() == 0) {
                joint.setPosition(.85);
                finished = !finished;
            } else {
                joint.setPosition(0);
                finished = !finished;
            }
            return false;
        }
    }

    public Action initializer() {
        return new initialize();
    }

    public Action angle() {
        return new moveArm();
    }

    public Action close() {
        return new grab();
    }

    public Action open() {return new prep();}
}
