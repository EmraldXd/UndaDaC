package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.linearSlides;

@Autonomous (name = "Yellow Auto")
public class RoadRunnerAutoOne extends OpMode {
    org.firstinspires.ftc.teamcode.action.linearSlides linearSlides = new linearSlides();
    org.firstinspires.ftc.teamcode.action.claw claw = new claw();
    ElapsedTime actionRuntime = new ElapsedTime();
    ElapsedTime autoRuntime = new ElapsedTime();
    TrajectorySequence Yellow;
    SampleMecanumDrive drive;
    boolean unrun;

    public void init() {
        linearSlides.init(this);
        claw.init(this);

        drive = new SampleMecanumDrive(hardwareMap);
    }

    public void start() {
        Yellow = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0.00)))
                .lineTo(new Vector2d(5, 0))
                .addDisplacementMarker(() -> unrun = false)
                .build();

        unrun = true;
    }

    @Override
    public void loop() {
        if(unrun) {
            drive.followTrajectorySequence(Yellow);
        }
    }
}
