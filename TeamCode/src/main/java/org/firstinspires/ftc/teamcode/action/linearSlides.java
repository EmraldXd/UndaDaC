package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class linearSlides {
    double MAX_POWER = .40;
    static final DecimalFormat df = new DecimalFormat("0.00");
    //Declare null
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor angleMotor;
    Telemetry telemetry;
    DigitalChannel touchSensor;
    DigitalChannel slideSensor;
    HardwareMap hardwareMap;
    double currentPosition;
    double currentAngle;
    double angleOffset;
    boolean downLastPressed;
    double MAX_FORWARD_DISTANCE = 3250; //Approximately max distance we can horizontally extend in ticks
    double maxExtend;
    double speedRatio;
    double slidesPosition;
    double lastReadPosition;
    double slidesOffset;
    boolean isStopped;
    boolean slideStopped;
    boolean request;
    double lastPower;
    double currentPower;


    public void init(@NonNull OpMode opMode){
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        //Initialize motors
        angleMotor = hardwareMap.get(DcMotor.class, "AngleMotor");
        rightSlide = hardwareMap.get(DcMotor.class, "RightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftSlide");
        //Initialize sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, "Touch Sensor");
        slideSensor = hardwareMap.get(DigitalChannel.class, "Slide Sensor");
        //Set Motor Direction
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set angle motor offset
        angleOffset = Math.abs(angleMotor.getCurrentPosition());
        slidesOffset = Math.abs(rightSlide.getCurrentPosition());
    }

    public void setSlides() {
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isStopped = false;
        request = false;
    }

    /**
     * This adds power to the motor that angles the slides.
     * @param up keeps track of if the up button on the dPad is hit
     * @param down keeps track of if the down button on the dPad is hit
     */
    public void angMotorPower(boolean up, boolean down){
        double y;
        //Set booleans to double values to work the linear slides
        y = (up ? -1 : 0) + (down ? 1 : 0);
        angleMotor.setPower(y * MAX_POWER);
        if(down || up) {
            angleMotor.setPower(y);
            downLastPressed = !downLastPressed;
        }
    }

    public void slidePower(double x) {
        slidesPosition = Math.abs(rightSlide.getCurrentPosition()) - slidesOffset;
        maxExtend = Math.abs(MAX_FORWARD_DISTANCE / Math.cos(ticksToRadians(angleMotor.getCurrentPosition())));
        speedRatio = ((maxExtend - slidesPosition) / 500);
        if(slideSensor.getState()) {
            if (slidesPosition - maxExtend < 0 || x > 0) {
                rightSlide.setPower((Math.abs(rightSlide.getCurrentPosition()) < 1250 && x > 0) ? 0.5 * x : x);
                leftSlide.setPower((Math.abs(rightSlide.getCurrentPosition()) < 1250 && x > 0) ? 0.5 * x : x); //We change the linear slide speed as it approaches the button to avoid physical damage
            } else {
                rightSlide.setPower(1);
                leftSlide.setPower(1);
            }
        } else if (!slideSensor.getState() && !slideStopped) {
            rightSlide.setPower(0); //Stop the linear slides when they hit the button
            rightSlide.setPower(0);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset the position for accurate measurement
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideStopped = true;
        } else if (slideStopped) {
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Start the motors back up
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(x < 0) {
                rightSlide.setPower(x);
                leftSlide.setPower(x);
            }
        }
        if(x < 0) {
            slideStopped = false;
        }
    }

    /**
     * This will take the amount of ticks the motor have moved and translates
     * it to radians, so we can calculate the farthest distance the linear slides can extend to.
     * @param rotation is how many ticks have been counted
     * @return returns the rotation in radians.
     */
    public double ticksToRadians(double rotation) {
        if(!touchSensor.getState() && !isStopped){
            angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isStopped = true;
        } else if(!touchSensor.getState() && isStopped) {
            angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if(isStopped) {
            isStopped = false;
        }
        currentAngle = Math.toRadians(90) * (rotation / 4000);
        return Math.abs(currentAngle);
    }

    public boolean liftTime() {
        return Math.abs(rightSlide.getCurrentPosition()) < 1750;
    }

    public void goToSpecimen(boolean pressed, boolean cancel){
        if(pressed) {
            request = true;
        } else if (cancel) {
            request = false;
        }
        if(request) {
            if (ticksToRadians(angleMotor.getCurrentPosition()) < Math.toRadians(50)) {
                angleMotor.setPower(-1);
            } else if(lastPower < currentPower || lastPower > currentPower) {
                request = false;
            } else {
                angleMotor.setPower(1);
            }
        }
        lastPower = currentPower;
        currentPower = angleMotor.getPower();
    }

    /**
     * This was used to find if the angle motor was following when using bevel gears before we swapped
     * to a worm gear. This would prevent us from getting penalized by giving the linear slides a
     * chance to autonomously retract.
     */
    /*public boolean angMotorFalling(double angleMotor) {
        if (lastReadPosition == 0) {
            lastReadPosition = Math.abs(angleMotor);
        }
        if (angleMotor - lastReadPosition < 0) {
            return true;
        } else {
            return false;
        }
    } */

    public void telemetryOutput() {
        telemetry.addData("Right Motor Power: ", df.format(rightSlide.getPower()));
        telemetry.addData("Right Motor Position: ", df.format(slidesPosition - slidesOffset));
        telemetry.addData("Max distance: ", maxExtend);
        telemetry.addData("Angle Motor Power: ", df.format(angleMotor.getPower()));
        telemetry.addData("Angle Motor Position: ", df.format(Math.toDegrees(ticksToRadians(angleMotor.getCurrentPosition()))) + " Degrees");
        telemetry.addData("angle motor position: ", angleMotor.getCurrentPosition());
        telemetry.addData("Position: ", df.format(angleMotor.getCurrentPosition()));
    }
}
