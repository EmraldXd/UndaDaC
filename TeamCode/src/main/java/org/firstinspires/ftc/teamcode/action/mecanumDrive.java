package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

/** This is an interface for this year's mecanum drive wheels. */
public class mecanumDrive {
    // CONSTRUCT
    static final DecimalFormat df = new DecimalFormat("0.00"); // for rounding
    // DECLARE NULL
    DcMotor fR;
    DcMotor fL;
    DcMotor bR;
    DcMotor bL;
    IMU imu;
    double fRPwr;
    double fLPwr;
    double bRPwr;
    double bLPwr;
    double heading;
    double adjustedX;
    double adjustedY;
    Telemetry telemetry;
    // DECLARE CUSTOM
    static double totalSpeed = 0.8; //This is to control the percent of energy being applied to the motors.
    double slowSpeed = 0.50; // x% of whatever speed totalSpeed is

    // METHODS
    /** Initializes the mecanum drive wheels.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        fR = hardwareMap.get(DcMotor.class, "Front Right");
        fL = hardwareMap.get(DcMotor.class, "Front Left");
        bR = hardwareMap.get(DcMotor.class, "Back Right");
        bL = hardwareMap.get(DcMotor.class, "Back Left");
        imu = hardwareMap.get(IMU.class, "imu");


        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Sets the mode of the motors to run WITHOUT encoders
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /** Enable or disable slow mode for the wheels.
     * @param enable True if you want to enable slow-mode. False if not.
     */
    public void slowMode(Boolean enable) {
        slowSpeed = enable ? 1.00 : 0.50;
    }

    /** Changes the maximum speed of the robot.
     * @apiNote By default this is 0.75
     */
    public void setMaxSpeed(double speed) {
        totalSpeed = speed;
    }

    /** Switches the wheels to run using encoders. */
    public void runUsingEncoder() {
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Switches the wheels to run without encoders. */
    public void runWithoutEncoder() {
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Switches the wheels to run to position mode. */
    public void runToPosition() {
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** Sends power to the mecanum wheels.
     * @param x Power to use for strafing. Passing in a gamepad joystick is best.
     * @param y Power to use for forwards/backwards movement. Passing in a gamepad joystick is best.
     * @param rot Power to use for rotating. Passing in a gamepad joystick is best.
     */
    public void setPower(double x, double y, double rot) {//rot is short for rotation
        x = x * 1.1;

        //Code to calculate motor power
        double ratio = Math.max((Math.abs(x) + Math.abs(y) + Math.abs(rot)), 1);
        fRPwr = (-x - y - rot) / ratio;
        fLPwr = (x - y + rot) / ratio;
        bRPwr = (x - y - rot) / ratio;
        bLPwr = (-x - y + rot) / ratio;

        fR.setPower(fRPwr * totalSpeed * slowSpeed);
        fL.setPower(fLPwr * totalSpeed * slowSpeed);
        bR.setPower(bRPwr * totalSpeed * slowSpeed);
        bL.setPower(bLPwr * totalSpeed * slowSpeed);
    }

    public void driversideDrive(double x, double y, double rot, boolean reset) {
        x = x * 1.1;

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.PI;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        fLPwr = ((-rotY + rotX + rot) / denominator) * slowSpeed;
        bLPwr = ((-rotY - rotX + rot) / denominator) * slowSpeed;
        fRPwr = ((-rotY - rotX - rot) / denominator) * slowSpeed;
        bRPwr = ((-rotY + rotX - rot) / denominator) * slowSpeed;

        fL.setPower(fLPwr);
        bL.setPower(bLPwr);
        fR.setPower(fRPwr);
        bR.setPower(bRPwr);

        if(reset) {
            imu.resetYaw();
        }
    }

    public void telemetryOutput() {
        // Power output
        telemetry.addData("fRMotorPwr", df.format(fRPwr));
        telemetry.addData("fLMotorPwr", df.format(fLPwr));
        telemetry.addData("bRMotorPwr", df.format(bRPwr));
        telemetry.addData("bLMotorPwr", df.format(bLPwr));
        // Ticks
        telemetry.addData("Front Left Ticks", fL.getCurrentPosition());
        telemetry.addData("Front Right Ticks", fR.getCurrentPosition());
        telemetry.addData("Back Left Ticks", bL.getCurrentPosition());
        telemetry.addData("Back Right Ticks", bR.getCurrentPosition());
        //Imu heading
        telemetry.addData("IMU heading: ", imu.getRobotYawPitchRollAngles());
        telemetry.addData("IMU heading 2: ", imu.getRobotAngularVelocity(AngleUnit.DEGREES));
    }
}