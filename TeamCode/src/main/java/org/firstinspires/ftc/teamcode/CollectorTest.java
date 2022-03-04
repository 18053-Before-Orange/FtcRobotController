/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static org.firstinspires.ftc.teamcode.Lift.CAP;
import static org.firstinspires.ftc.teamcode.Lift.CAPPING_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.CAP_PICKUP;
import static org.firstinspires.ftc.teamcode.Lift.CAP_PICKUP_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.COLLECTION;
import static org.firstinspires.ftc.teamcode.Lift.COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY_1_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY_2_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DELIVERY_3_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DUCK_APPROACH;
import static org.firstinspires.ftc.teamcode.Lift.DUCK_APPROACH_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.DUCK_SPIN;
import static org.firstinspires.ftc.teamcode.Lift.DUCK_SPIN_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Lift.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Lift.LEVEL_3;
import static org.firstinspires.ftc.teamcode.Lift.MOVE;
import static org.firstinspires.ftc.teamcode.Lift.MOVE_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.PARK;
import static org.firstinspires.ftc.teamcode.Lift.PARK_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.TRANSIT;

@Disabled
@TeleOp(name="CollectorTest", group="18053")
public class CollectorTest extends LinearOpMode {
    //Initializes joystick storage variables
    private boolean isFreightDetected = false;
    private boolean collectorOn = false;
    private boolean collectorButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        CollectorMotor collector = new CollectorMotor(hardwareMap, telemetry, this);

        waitForStart();


        while (opModeIsActive()) {
            if (!isFreightDetected) {
                isFreightDetected = detectFreight(collector);
            }
            if (gamepad1.right_trigger > 0.5 && !isFreightDetected) {
                collector.speed(0.75);
            } else {
                collector.speed(0);
            }

            showFreightColors(collector);
            telemetry.update();

        }
    }

    private boolean detectFreight(CollectorMotor collector) {
        NormalizedRGBA colors = collector.getFreightColor(collector.freightDetector);
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] > 10) {
            return true;
        } else {
            return false;
        }
    }

    public void showFreightColors(CollectorMotor collector) {
        NormalizedRGBA colors = collector.getFreightColor(collector.freightDetector);
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.addData("Distance (cm)", "%.3f", collector.getFreightDistance(collector.freightDetector));
    }
}