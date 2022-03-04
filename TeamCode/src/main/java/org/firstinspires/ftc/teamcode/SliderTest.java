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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Disabled
@TeleOp(name="SliderTest", group="18053")
public class SliderTest extends LinearOpMode {
    //Initializes joystick storage variables
    private boolean isFreightDetected = false;
    private boolean collectorOn = false;
    private boolean collectorButtonPressed = false;
    private int liftPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CollectorMotor collector = new CollectorMotor(hardwareMap, telemetry, this);
        SliderMotor slider = new SliderMotor(hardwareMap, telemetry, this);
        Lift lift = new Lift(hardwareMap, telemetry, this);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.5) {
                collector.speed(0.75);
            } else {
                collector.speed(0);
            }

            if (gamepad1.dpad_up) {
                liftPosition += 100;
            } else if (gamepad1.dpad_down) {
                liftPosition = liftPosition - 100;
            }

            if (gamepad1.dpad_left) {
                slider.speed(1.0);
            } else if (gamepad1.dpad_right) {
                slider.speed(-1.0);
            } else {
                slider.speed(0);
            }

            lift.runToPosition(liftPosition);

//            showValues(slider);
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

//    public void showValues(SliderMotor slider) {
//        telemetry.addLine()
//                .addData("Slider", slider.getSliderPosition())
//                .addData("Slider Center", slider.isCenter());
//    }
}