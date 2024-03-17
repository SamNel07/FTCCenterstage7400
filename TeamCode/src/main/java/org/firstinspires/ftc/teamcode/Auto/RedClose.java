package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.RedPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Red", name = "RedClose")
public class RedClose extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    public RedPipeline pipeline;

    public Servo pl = null;



    @Override
    public void runOpMode() {

        pl = hardwareMap.get(Servo.class, "pl");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(14.5, -61.5, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Left = drive.trajectorySequenceBuilder(startPose)
                //.strafeRight(18)
                .lineToConstantHeading(new Vector2d(14.5, -43.5))
                .lineToSplineHeading(new Pose2d(8.7, -33, Math.toRadians(-100)))
                //.strafeLeft(6)
                .lineToConstantHeading(new Vector2d(15.2, -34))
                .lineToSplineHeading(new Pose2d(51.65, -29.5, Math.toRadians(176)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.55)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                .lineToConstantHeading(new Vector2d(45.9, -29))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeLeft(31)
                .lineToConstantHeading(new Vector2d(45.9, -62))
                //.back(14)
                .lineToConstantHeading(new Vector2d(59.9, -62))
                .build();


        TrajectorySequence Center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15.5, -30))
                //.strafeLeft(5)
                .lineToLinearHeading(new Pose2d(15.5, -34.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(50.7, -34.5, Math.toRadians(178)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                .lineToLinearHeading(new Pose2d(45.3, -35, Math.toRadians(180)))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeLeft(24)
                .lineToLinearHeading(new Pose2d(45.3, -61, Math.toRadians(180)))
                //.back(14)
                .lineToLinearHeading(new Pose2d(59.3, -61, Math.toRadians(180)))
                .build();

        TrajectorySequence Right = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(23, -41.5), Math.toRadians(180))
                //.strafeLeft(6)
                .lineToLinearHeading(new Pose2d(23, -47.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(50.9, -40.5))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.6)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                .lineToConstantHeading(new Vector2d(45, -40.5))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeLeft(17.5)
                .lineToConstantHeading(new Vector2d(45, -58))
                //.back(14)
                .lineToConstantHeading(new Vector2d(59, -58))
                .build();


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMontiorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new RedPipeline();
        webcam1.setPipeline(pipeline);

        webcam1.setMillisecondsPermissionTimeout(2500);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
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
