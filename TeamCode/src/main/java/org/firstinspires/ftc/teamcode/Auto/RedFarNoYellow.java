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

@Autonomous(group = "Red", name = "RedFarNoYellow")
public class RedFarNoYellow extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    public RedPipeline pipeline;

    public Servo pl = null;



    @Override
    public void runOpMode() {

        pl = hardwareMap.get(Servo.class, "pl");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-37.5, -61.5, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Left = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-45.5, -41.5), Math.toRadians(180))
                //.strafeLeft(6)
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-46, -47.5))
                //.back(11)
                .lineToConstantHeading(new Vector2d(-35, -47.5))
                .lineToConstantHeading(new Vector2d(-34, -10))
                .lineToConstantHeading(new Vector2d(20, -15))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(50, -20,Math.toRadians(170)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                /*.lineToSplineHeading(new Pose2d(47, -35,Math.toRadians(170)))
                .addTemporalMarker(() -> pl.setPosition(0))
                //.strafeRight(14)
                .lineToSplineHeading(new Pose2d(47, -15,Math.toRadians(170)))
                //.back(16)
                .lineToSplineHeading(new Pose2d(63, -15,Math.toRadians(170)))*/
                .build();


        TrajectorySequence Center = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-34, -31))
                //.strafeLeft(5)
                .lineToConstantHeading(new Vector2d(-34, -36))
                .lineToConstantHeading(new Vector2d(-52, -34))
                .lineToConstantHeading(new Vector2d(-50, -9))
                .lineToLinearHeading(new Pose2d(0, -14, Math.toRadians(180)))
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(40, -14, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(50, -20, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                //.lineToLinearHeading(new Pose2d(45.5, -40, Math.toRadians(180)))
                .addTemporalMarker(() -> pl.setPosition(0))
                .waitSeconds(2)
                /*//.strafeRight(21)
                .lineToLinearHeading(new Pose2d(45.5, -19, Math.toRadians(180)))
                //.back(14)
                .lineToLinearHeading(new Pose2d(59.5, -19, Math.toRadians(180)))*/
                .build();

        TrajectorySequence Right = drive.trajectorySequenceBuilder(startPose)
                //.strafeRight(25)
                .lineToSplineHeading(new Pose2d(-37.5, -39, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-32, -34.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-34.5, -34, Math.toRadians(90)))
                //.strafeLeft(6)
                .lineToSplineHeading(new Pose2d(-39, -34,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-35, -13))
                .lineToConstantHeading(new Vector2d(20, -13))
                .waitSeconds(5)
                .lineToSplineHeading(new Pose2d(50, -20,Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> pl.setPosition(0.5))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> pl.setPosition(1))
                .waitSeconds(2)
                //.forward(5)
                //.lineToSplineHeading(new Pose2d(63, -51.5,Math.toRadians(170)))
                .addTemporalMarker(() -> pl.setPosition(0))
                .waitSeconds(2)
                //.strafeRight(14)
                //.lineToSplineHeading(new Pose2d(45, -26.5,Math.toRadians(180)))
                //.back(16)
                //.lineToSplineHeading(new Pose2d(61, -26.5,Math.toRadians(180)))
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
