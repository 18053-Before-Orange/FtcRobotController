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
@Disabled
public class BlueAuto extends LinearOpMode
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
        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        int deliveryPosition = DELIVERY_1_POSITION;

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-36, 58))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-64, 54, Math.toRadians(125)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-14, 37, Math.toRadians(270)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(12, 30, Math.toRadians(200)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(54, 57, Math.toRadians(200)))
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

            slider.resetEncoders();
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
                break;
            }

            case RIGHT:
            {
                deliveryPosition = DELIVERY_3_POSITION;
                break;
            }

            case CENTER:
            {
                deliveryPosition = DELIVERY_2_POSITION;
                break;
            }
        }

        telemetry.addData("Slider", slider.getSliderPosition());
        telemetry.update();


        lift.runToPosition(DUCK_APPROACH_POSITION);
        drive.followTrajectory(traj1);
        duck.down();
        duck.spinRight();
        drive.followTrajectory(traj2);
        sleep(1500);
        duck.spinStop();
        duck.up();
        drive.followTrajectory(traj3);

        lift.runToPosition(deliveryPosition);
        slider.runToPositionNoProtection(125);

        sleep(2000);
        collector.egressSpeed(0.99);
        collector.speed(0.55);
        sleep(1000);
        collector.speed(0);
        collector.egressSpeed(0);
        slider.runToPosition(0);
        lift.runToPosition(MOVE_POSITION);


        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        lift.runToPosition(COLLECTION_POSITION);

        Trajectory trajCollect = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(6)
                .build();

        drive.followTrajectory(trajCollect);

        while(!collector.detectFreight()) {
            collector.speed(0.65);
        }
        collector.speed(0);
        lift.runToPosition(MOVE_POSITION);

        Trajectory trajDeliver = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(220)))
                .build();
        drive.followTrajectory(trajDeliver);

        lift.runToPosition(DELIVERY_3_POSITION);
        slider.runToPositionNoProtection(125);
        sleep(2000);
        collector.egressSpeed(0.99);
        collector.speed(0.55);
        sleep(1000);
        collector.speed(0);
        collector.egressSpeed(0);
        slider.runToPosition(10);
        lift.runToPosition(MOVE_POSITION);

        Trajectory trajPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(55, 45, Math.toRadians(190)))
                .build();
        drive.followTrajectory(trajPark);

        lift.runToPosition(PARK_POSITION);




        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}
