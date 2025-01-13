package org.firstinspires.ftc.teamcode.customAction;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightRR {

    public Pose2d currentPose;
    public Pose3D botpose;
    Limelight3A limelight;

    class init implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            limelight.pipelineSwitch(9);
            limelight.start();
            return false;
        }
    }
    class localizer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double currentX = currentPose.position.component1();
            return false;
        }
    }
}
