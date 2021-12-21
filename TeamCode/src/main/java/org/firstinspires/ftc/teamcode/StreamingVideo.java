package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class StreamingVideo extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AZKkf2j/////AAABmTXbwO4/1EjKlK73j4Ks7io8ld5sXAm4JFoiFVl3ht0iBGTWMiQAlVNaeCgBTq+MPDGSMdw85lJ0mJ2J//MAEnDPakThYtNKJvH+8+j7PN/3yZrIg4dYoZe3uBWImD2i+tY+9IS4IGNOT1zoBVDiUeHFIL7goDQB27gG9mfwupifmDA0FvKp7AacOpGrPn/GLL8t5V3p1nqbuX7e9JTxovC6Hik49A/EPJ2vLWpW8jsePlFTO1VfylmUYSl+3WioHqinTd3k+HMLIzXpVvoqke0KOG+XF32OYy3ixiGbsKrRrAd+/VqZNdmJIl1NiR+yxW5iVesQ0TJlyrmk2PmwdmMgsLjUZmJcfGqWRbNUalQz";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}