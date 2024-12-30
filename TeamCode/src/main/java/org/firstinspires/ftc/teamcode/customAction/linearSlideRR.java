package org.firstinspires.ftc.teamcode.customAction;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class linearSlideRR {
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor angleMotor;


    public linearSlideRR(HardwareMap hardwareMap) {
        angleMotor = hardwareMap.get(DcMotor.class, "AngleMotor");
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public class AngleSlidesUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);;
                angleMotor.setPower(1);
            }

            if(angleMotor.getCurrentPosition() >= 4000) {
                angleMotor.setPower(0);
            }
            return angleMotor.getCurrentPosition() <= 4000;
        }
    }

    public class AngleSlidesDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);;
                angleMotor.setPower(-1);
            }

            if(angleMotor.getCurrentPosition() <= 0) {
                angleMotor.setPower(0);
            }
            return angleMotor.getCurrentPosition() >= 0;
        }
    }







    //The callable actions
    public Action angleSlidesUp() {
        return new AngleSlidesUp();
    }

    public Action angleSlidesDown() {
        return new AngleSlidesDown();
    }
}
