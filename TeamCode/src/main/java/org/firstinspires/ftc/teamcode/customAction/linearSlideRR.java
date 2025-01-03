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
            if(!initialized) {
                this.pos = runPosition;
                this.pow = runPower;
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (rightSlide.getCurrentPosition() < pos) {
                    rightSlide.setPower(pow);
                    leftSlide.setPower(pow);
                } else if (rightSlide.getCurrentPosition() > pos) {
                    rightSlide.setPower(-pow);
                    leftSlide.setPower(-pow);
                } else {
                    return true;
                }
                initialized = !initialized;
            }

                if(rightSlide.getCurrentPosition() >= 2000 - 25 || rightSlide.getCurrentPosition() <= 2000 + 25) {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                }

                return (rightSlide.getCurrentPosition() >= 2000 - 25 || rightSlide.getCurrentPosition() <= 2000 + 25);
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
        runPower = power;
        runPosition = position;
        return new RunToPosition();
    }
}