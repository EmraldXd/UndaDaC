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
    Servo ejecter;
    double placeholder;
    public clawRR(HardwareMap hardwareMap) {
        joint = hardwareMap.get(Servo.class, "Joint");
        claw = hardwareMap.get(Servo.class, "Claw");
        angler = hardwareMap.get(Servo.class, "Angler");
        ejecter = hardwareMap.get(Servo.class, "Eject");
    }

    /** This initializes the claw for RoadRunner usage */
    public class initialize implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0);
            joint.setPosition(0);
            angler.setPosition(0);
            ejecter.setPosition(0);
            if(claw.getPosition() == 0 && angler.getPosition() == 0 && joint.getPosition() == 0){
                return false;
            } else {
                return true;
            }
        }
    }

    /** This causes the claw to grab the samples in auto */
    public class grab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0);
            return false;
        }
    }

    /** This sets up the claw to grab samples in auto by opening the claw */
    public class prep implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0.4);
            return false;
        }
    }

    /** This moves the arm to grab samples */
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

    public class eject implements Action {
        private boolean finished;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ejecter.setPosition(1);
            if(ejecter.getPosition() == 1) {
                return false;
            }
            return true;
        }
    }

    public class reset implements Action {
        private boolean finished;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ejecter.setPosition(0);
            if(ejecter.getPosition() == 0) {
                return false;
            }
            return true;
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

    public Action ejectSpecimen() {
        return new eject();
    }

    public Action resetEject() {
        return new reset();
    }
}
