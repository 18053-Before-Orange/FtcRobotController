package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.SkystoneDeterminationExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.Lift.COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY_1_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY_2_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY_3_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DUCK_APPROACH_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DUCK_SPIN_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.MOVE_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.PARK_POSITION;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous
//@Disabled
public class BlueAutoWarehouse extends LinearOpMode
{
    OpenCvWebcam webcam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        Lift lift = new Lift(hardwareMap, telemetry, this);
        CollectorMotor collector = new CollectorMotor(hardwareMap, telemetry, this);
        SliderMotor slider = new SliderMotor(hardwareMap, telemetry, this);
        DuckRoller duck = new DuckRoller(hardwareMap, telemetry, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        int deliveryPosition = DELIVERY_1_POSITION;
        int sliderPosition = slider.SLIDER_1_POSITION;

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, 58))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-14, 46, Math.toRadians(270)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(12, 58, Math.toRadians(180)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(42, 58, Math.toRadians(180)))
                .build();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        slider.resetEncoders();
        lift.resetEncoders();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.addData("Left", pipeline.getLeft());
            telemetry.addData("Center", pipeline.getCenter());
            telemetry.addData("Right", pipeline.getRight());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {

            case LEFT:
            {
                deliveryPosition = DELIVERY_1_POSITION;
                sliderPosition = slider.SLIDER_1_POSITION;
                break;
            }

            case RIGHT:
            {
                deliveryPosition = DELIVERY_3_POSITION;
                sliderPosition = slider.SLIDER_3_POSITION;
                break;
            }

            case CENTER:
            {
                deliveryPosition = DELIVERY_2_POSITION;
                sliderPosition = slider.SLIDER_2_POSITION;
                break;
            }
        }

        lift.runToPosition(DUCK_APPROACH_POSITION);
        drive.followTrajectory(traj1);
        lift.runToPosition(deliveryPosition);
        slider.runToPositionNoProtection(sliderPosition);
        drive.followTrajectory(traj2);
        collector.egressSpeed(0.99);
        collector.speed(0.9);
        sleep(2000);
        collector.speed(0);
        collector.egressSpeed(0);
        slider.runToPositionNoProtection(10);
        sleep(1000);
        lift.runToPosition(MOVE_POSITION);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

        lift.runToPosition(COLLECTION_POSITION);

        Trajectory trajCollect = drive.trajectoryBuilder(traj4.end())
                .back(6)
                .build();

        collector.speed(0.9);

        while(!collector.detectFreight()) {
            drive.followTrajectory(trajCollect);
            trajCollect = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(1)
                    .build();
            sleep(500);
        }
        collector.speed(0);

        while (!slider.isCenter()) {
            slider.runToPositionNoProtection(0);
            telemetry.addData("Slider Pos: ", slider.getSliderPosition());
            telemetry.update();
        }
        lift.runToPosition(MOVE_POSITION);

        Trajectory trajCheckpoint = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(12, 58, Math.toRadians(180)))
                .build();
        drive.followTrajectory(trajCheckpoint);

        lift.runToPosition(DELIVERY_3_POSITION);
        slider.runToPositionNoProtection(slider.SLIDER_3_POSITION);


        Trajectory trajDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-14, 46, Math.toRadians(270)))
                .build();
        drive.followTrajectory(trajDeliver);

        collector.egressSpeed(0.99);
        collector.speed(0.9);
        sleep(2000);
        collector.speed(0);
        collector.egressSpeed(0);
        slider.runToPositionNoProtection(10);
        sleep(1000);
        lift.runToPosition(MOVE_POSITION);

        trajCheckpoint = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(12, 58, Math.toRadians(180)))
                .build();
        drive.followTrajectory(trajCheckpoint);

        Trajectory trajPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(42, 58, Math.toRadians(180)))
                .build();
        drive.followTrajectory(trajPark);
        lift.runToPosition(PARK_POSITION);
//
//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive())
//        {
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
    }
}