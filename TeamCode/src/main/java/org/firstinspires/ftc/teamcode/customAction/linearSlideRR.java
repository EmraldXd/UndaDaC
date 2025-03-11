package org.firstinspires.ftc.teamcode.customAction;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class linearSlideRR {
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor angleMotorA;
    DcMotor angleMotorB;
    public double runPosition;
    public double runPower;


    public linearSlideRR(HardwareMap hardwareMap) {
        angleMotorA = hardwareMap.get(DcMotor.class, "AngleMotorA");
        angleMotorB = hardwareMap.get(DcMotor.class, "ParEncoder");
        rightSlide = hardwareMap.get(DcMotor.class, "RightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftSlide");
        angleMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //angleMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** This angles the slides upward to score samples or specimens */
    public class AngleSlidesUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(angleMotorA.getCurrentPosition() >= 2980) {
                angleMotorA.setPower(0);
                angleMotorB.setPower(0);
                return false;
            }

            angleMotorA.setPower(.75);
            angleMotorB.setPower(.75);
            return true;
        }
    }

    /** This returns the slides to their 0º position */
    public class AngleSlidesDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(angleMotorA.getCurrentPosition() <= 0) {
                angleMotorA.setPower(0);
                angleMotorB.setPower(0);
                return false;
            }

            angleMotorB.setPower(-1);
            angleMotorA.setPower(-1);
            return true;
        }
    }

    /** This extends the linear slides to the height to hang specimens on the high rung */
    public class RunToHighRung implements Action{
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put("a_rightPos", rightSlide.getCurrentPosition());

            if(rightSlide.getCurrentPosition() >= 2675) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
                return false;
            }
            rightSlide.setPower(1);
            leftSlide.setPower(1);
            return true;
            //return rightSlide.getCurrentPosition() <= 2695;
        }
    }

    /** This has the linear slides extend to the distance to place samples in the high baskets */
    public class RunToHighBasket implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(rightSlide.getCurrentPosition() >= 4900) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
                return false;
            }

            rightSlide.setPower(1);
            leftSlide.setPower(1);
            return true;
        }
    }

    /**
     * This extends the linear slides to the position to pick up samples of the spike marks on
     * the ground.
     */
    public class collectSample implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(rightSlide.getCurrentPosition() >= 2000) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
                return false;
            }

            rightSlide.setPower(1);
            leftSlide.setPower(1);
            return true;
        }
    }

    /** This fully retracts the linear slides */
    public class runToZero implements Action {
        public boolean notInit = true;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(rightSlide.getCurrentPosition() <= 100) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
                return false;
            }

            rightSlide.setPower(-1);
            leftSlide.setPower(-1);
            return true;
        }
    }

    /** This slightly raises the linear slides to line up with the specimen hook */
    public class takeSpecimen implements Action {
        public boolean notInit = true;
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(rightSlide.getCurrentPosition() >= 350) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
                return false;
            }

            rightSlide.setPower(1);
            leftSlide.setPower(1);
            return true;
        }
    }

    /**
     * Not used anymore.
     *
     * This code is used to get the linear slides to the angle they need for our main claw to hang
     * specimens on the high rung.

    public class AngleSlidesSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(angleMotor.getCurrentPosition() >= 3900) {
                angleMotor.setPower(0);
                return false;
            }

            angleMotor.setPower(1);
            return true;
        }
    }

     */



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

    public Action take() {
        return new takeSpecimen();
    }

    //public Action specimenAngle(){return new AngleSlidesSpecimen();}
}