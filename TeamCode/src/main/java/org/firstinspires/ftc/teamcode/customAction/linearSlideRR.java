package org.firstinspires.ftc.teamcode.customAction;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class linearSlideRR {
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor angleMotor;
    public double runPosition;
    public double runPower;


    public linearSlideRR(HardwareMap hardwareMap) {
        angleMotor = hardwareMap.get(DcMotor.class, "AngleMotor");
        rightSlide = hardwareMap.get(DcMotor.class, "RightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftSlide");
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public class AngleSlidesUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);;
                angleMotor.setPower(1);
                initialized = true;
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
                initialized = true;
            }

            if(angleMotor.getCurrentPosition() <= 0) {
                angleMotor.setPower(0);
            }
            return angleMotor.getCurrentPosition() >= 0;
        }
    }

    public class RunToPosition implements Action{
        private boolean initialized = false;
        private double pos;
        private double pow;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(initialized) {
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setPower(0.5);
                leftSlide.setPower(0.5);
                initialized = !initialized;
            }
            if(Math.abs(rightSlide.getCurrentPosition()) > 2000) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                return false;
            } else {
                return true;
            }
        }
    }

    //The callable actions
    public Action angleSlidesUp() {
        return new AngleSlidesUp();
    }

    public Action angleSlidesDown() {
        return new AngleSlidesDown();
    }

    public Action runToPosition(double position, double power) {
        return new RunToPosition();
    }
}