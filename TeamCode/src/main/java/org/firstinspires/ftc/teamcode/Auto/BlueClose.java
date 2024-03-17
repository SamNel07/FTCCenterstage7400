package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.BluePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Blue", name = "BlueClose")
public class BlueClose extends LinearOpMode {

    OpenCvWebcam webcam2 = null;
    public BluePipeline pipeline;

    public Servo pl = null;



    @Override
    public void runOpMode() {

        pl = hardwareMap.get(Servo.class, "pl");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(14.5, 61.5, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Left = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(23.3, 41.5), Math.toRadians(180))
                //.strafeRight(6)
                .lineToConstantHeading(new Vector2d(23, 47.5))
                .lineToConstantHeading(new Vector2d(50.8, 44.5))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                .lineToConstantHeading(new Vector2d(45.5, 44.5))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeRight(15)
                .lineToConstantHeading(new Vector2d(45.5, 59.5))
                //.back(14)
                .lineToConstantHeading(new Vector2d(59.5, 59.5))
                .build();


        TrajectorySequence Center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(16, 31))
                //.strafeRight(5)
                .lineToConstantHeading(new Vector2d(16, 36))
                .lineToLinearHeading(new Pose2d(51.9, 39, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                .lineToLinearHeading(new Pose2d(45.5, 38.5, Math.toRadians(180)))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeRight(21)
                .lineToLinearHeading(new Pose2d(45.5, 60, Math.toRadians(180)))
                //.back(14)
                .lineToLinearHeading(new Pose2d(59.5, 60, Math.toRadians(180)))
                .build();



        TrajectorySequence Right = drive.trajectorySequenceBuilder(startPose)
                //.strafeLeft(18)
                .lineToLinearHeading(new Pose2d(14.5, 43.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(9, 32, Math.toRadians(100)))
                //.strafeRight(6)
                .lineToConstantHeading(new Vector2d(15, 32))
                .lineToSplineHeading(new Pose2d(52.25, 32.9,Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                .lineToConstantHeading(new Vector2d(47, 32))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeRight(29)
                .lineToConstantHeading(new Vector2d(47, 61.5))
                //.back(14)
                .lineToConstantHeading(new Vector2d(64, 61.5))
                .build();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam2");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMontiorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new BluePipeline();
        webcam2.setPipeline(pipeline);

        webcam2.setMillisecondsPermissionTimeout(2500);

        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam2.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }
        });




        waitForStart();

        if(isStopRequested()) return;



        switch (pipeline.position) {
            case LEFT: {

                drive.followTrajectorySequence(Left);

            } break;

            case CENTER: {

                drive.followTrajectorySequence(Center);

            } break;

            case RIGHT: {

                drive.followTrajectorySequence(Right);

            } break;
        }
    }

}
