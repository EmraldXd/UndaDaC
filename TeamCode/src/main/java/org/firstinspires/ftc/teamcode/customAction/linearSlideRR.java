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

            if(angleMotor.getCurrentPosition() >= 3800) {
                angleMotor.setPower(0);
            }
            return angleMotor.getCurrentPosition() <= 3800;
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

    public class RunToHighRung implements Action{

        public boolean notInit = true;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(notInit) {
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setPower(1);
                leftSlide.setPower(1);
                notInit = !notInit;
            }

            if(rightSlide.getCurrentPosition() >= 2695) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
            }

            return rightSlide.getCurrentPosition() <= 2695;
        }
    }

    public class RunToHighBasket implements Action {
        public boolean notInit = true;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(notInit) {
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setPower(1);
                leftSlide.setPower(1);
                notInit = !notInit;
            }

            if(rightSlide.getCurrentPosition() >= 4300) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
            }

            return rightSlide.getCurrentPosition() <= 4300;
        }
    }

    public class collectSample implements Action {
        public boolean notInit = true;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(notInit) {
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setPower(1);
                leftSlide.setPower(1);
                notInit = !notInit;
            }

            if(rightSlide.getCurrentPosition() >= 2000) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
            }

            return rightSlide.getCurrentPosition() <= 2000;
        }
    }

    public class runToZero implements Action {
        public boolean notInit = true;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(notInit) {
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setPower(-1);
                leftSlide.setPower(-1);
                notInit = !notInit;
            }

            if(rightSlide.getCurrentPosition() <= 100) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
            }

            return rightSlide.getCurrentPosition() >= 100;
        }
    }

    //The callable actions
    public Action angleSlidesUp() {
        return new AngleSlidesUp();
    }

    public Action angleSlidesDown() {
        return new AngleSlidesDown();
    }

    public Action runToHighRung() {
        return new RunToHighRung();
    }

    public Action runToHighBasket(){
        return new RunToHighBasket();
    }

    public Action collectPos() {
        return new collectSample();
    }

    public Action home() {
        return new runToZero();
    }
}