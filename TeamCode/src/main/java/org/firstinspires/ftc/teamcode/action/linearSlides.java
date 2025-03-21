package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class linearSlides {
    double MAX_POWER = .40;
    static final DecimalFormat df = new DecimalFormat("0.00");
    //Declare null
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor angleMotorA;
    DcMotor angleMotorB;
    Telemetry telemetry;
    DigitalChannel touchSensor;
    DigitalChannel slideSensor;
    HardwareMap hardwareMap;

    IMU imu;
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
        angleMotorA = hardwareMap.get(DcMotor.class, "AngleMotorA");
        angleMotorB = hardwareMap.get(DcMotor.class, "ParEncoder");
        rightSlide = hardwareMap.get(DcMotor.class, "RightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "LeftSlide");

        //Initialize sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, "Touch Sensor");
        slideSensor = hardwareMap.get(DigitalChannel.class, "Slide Sensor");

        //Set Motor Direction
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotorA.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotorB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Motor Behaviors
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Stop the motors to reset them to 0
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //We do not reset AngleMotorB because it does not need to return data

        //Set angle motor offset
        angleOffset = Math.abs(angleMotorA.getCurrentPosition());
        slidesOffset = Math.abs(rightSlide.getCurrentPosition());

        //Initialize IMU for the pitch of the robot
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void setSlides() {
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        angleMotorA.setPower(y);
        angleMotorB.setPower(y);
    }

    public void slidePower(double x) {
        slidesPosition = Math.abs(rightSlide.getCurrentPosition()) - slidesOffset;
        /*
        *   Written out, this is the remaining horizontal distance (24 inches in ticks) multiplied
        *   by the secant of the angle made by the robot pitch and slide angle.
        *   maxExtend = 24sec(Θ)
        */
        maxExtend = Math.abs(MAX_FORWARD_DISTANCE / (Math.cos(ticksToRadians(angleMotorA.getCurrentPosition()) + Math.abs(imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS)))));
        speedRatio = ((maxExtend - slidesPosition) / 500);
        if(slideSensor.getState()) {
            if (slidesPosition - maxExtend < 0 || x > 0) {
                rightSlide.setPower((Math.abs(rightSlide.getCurrentPosition()) < 1250 && x > 0) ? 0.5 * x : x);
                leftSlide.setPower((Math.abs(rightSlide.getCurrentPosition()) < 1250 && x > 0) ? 0.5 * x : x); //We change the linear slide speed as it approaches the button to avoid physical damage
            } else {
                rightSlide.setPower(.5);
                leftSlide.setPower(.5);
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
            angleMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            angleMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isStopped = true;
        } else if(!touchSensor.getState() && isStopped) {
            angleMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            angleMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if(isStopped) {
            isStopped = false;
        }
        currentAngle = Math.toRadians(90) * (rotation / 2100);
        return Math.abs(currentAngle);
    }

    public boolean liftTime() {
        return Math.abs(rightSlide.getCurrentPosition()) < 1800;
    }

    /**
     * This code is completely unused in newer designs. Previously it was made to allow the linear
     * slides to autonomously go the specified angle to hang specimens with a single button press.
     * Since we added the specimen claw, this chunk of code remains obsolete.
     * @param pressed is when the button is pressed to begin angling.
     * @param cancel is when literally ANY OTHER button to move the angle motor is pressed.
     */
    public void goToSpecimen(boolean pressed, boolean cancel){
        if(pressed) {
            request = true;
        } else if (cancel) {
            request = false;
        }
        if(request) {
            if (ticksToRadians(angleMotorA.getCurrentPosition()) < Math.toRadians(50)) {
                angleMotorA.setPower(-1);
            } else if(lastPower < currentPower || lastPower > currentPower) {
                request = false;
            } else {
                angleMotorA.setPower(1);
            }
        }
        lastPower = currentPower;
        currentPower = angleMotorA.getPower();
    }

    /**
     * This is being used in order to send back to the claw that the linear slides have either raised
     * or lowered to the right position to move the claw out of the way of objects, reducing the chance
     * of damaging the field.
     */

    public String moveUpOrDown() {
        if(ticksToRadians(angleMotorA.getCurrentPosition()) >=  Math.toRadians(60) && ticksToRadians(angleMotorA.getCurrentPosition()) <= Math.toRadians(70)) {
            if(angleMotorA.getPower() > 0) {
                return "Lower";
            } else if(angleMotorA.getPower() < 0) {
                return "Raise";
            } else {
                return "Nuhuh";
            }
        } else {
            return "nope";
        }
    }

    public double current() {
        return ticksToRadians(angleMotorA.getCurrentPosition());
    }

    public void slideCheck() {
        if(angleMotorA.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            angleMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(angleMotorB.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            angleMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(rightSlide.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(leftSlide.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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

    public void resetSlides(boolean pressed) {
        if(pressed) {
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void telemetryOutput() {
        telemetry.addData("Right Motor Power: ", df.format(rightSlide.getPower()));
        telemetry.addData("Right Motor Position: ", df.format(slidesPosition - slidesOffset));
        telemetry.addData("Max distance: ", maxExtend);
        telemetry.addData("Angle Motor Power: ", df.format(angleMotorA.getPower()));
        telemetry.addData("Angle Motor Position: ", df.format(Math.toDegrees(ticksToRadians(angleMotorA.getCurrentPosition()))) + " Degrees");
        telemetry.addData("angle motor position: ", angleMotorA.getCurrentPosition());
        telemetry.addData("Position: ", df.format(angleMotorA.getCurrentPosition()));
    }
}
