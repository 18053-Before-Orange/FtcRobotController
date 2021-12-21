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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@TeleOp(name="Orange", group="18053")
public class Orange extends LinearOpMode {
    //Initializes joystick storage variables
    private double leftStickX, leftStickY, rightStickX, rightStickY;
    private double lightValue;
    private boolean isFreightDetected = false;
    private boolean isForwardStopDetected = false;
    private boolean isBackStopDetected = false;
    private boolean isCenterStopDetected = false;
    private boolean collectorOn = false;
    private boolean collectorButtonPressed = false;
    private boolean ejectorOn = false;
    private boolean ejectorButtonPressed = false;
    private boolean sliderForward = false;
    private boolean sliderForwardEnabled = false;
    private boolean sliderForwardPressed = false;
    private boolean sliderBack = false;
    private boolean sliderBackEnabled = false;
    private boolean sliderBackPressed = false;
    private int liftPosition = 0;
    private String liftSetting = PARK;
    private MagneticSensor magSensor;

    private String deliveryPosition = LEVEL_3;
    private boolean robotModePressed = false;
    private boolean sliderOut = false;
    private boolean sliderPressed = false;
    private int deliveryPositionCounter = 0;
    private boolean deliveryPositionPressed = false;
    private boolean deployDuckRoller = false;
    private boolean deployDuckRollerPressed = false;
    private boolean deployClaw = false;
    private boolean deployClawPressed = false;
    private boolean clawLiftUp = false;
    private boolean clawLiftUpPressed = false;
    private boolean clawOpen = false;
    private boolean clawOpenPressed = false;
    private boolean levelChangePressed = false;

    private static final RevBlinkinLedDriver.BlinkinPattern GREEN_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
    private static final RevBlinkinLedDriver.BlinkinPattern RED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
    private static final RevBlinkinLedDriver.BlinkinPattern PURPLE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
    private static final RevBlinkinLedDriver.BlinkinPattern GOLD_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    private static final RevBlinkinLedDriver.BlinkinPattern BLUE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
    private static final double threshold = 0.1;
    private static final String DELIVER = "DELIVER";
    private static final String COLLECT = "COLLECT";
    private String robotMode = DELIVER;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        Lift lift = new Lift(hardwareMap, telemetry, this);
        Collector collector = new Collector(hardwareMap, telemetry, this);
        Slider slider = new Slider(hardwareMap, telemetry, this);
        DuckRoller duck = new DuckRoller(hardwareMap, telemetry, this);
        Claw claw = new Claw(hardwareMap, telemetry, this);
        Light light = new Light(hardwareMap, telemetry, this);

//        magSensor = hardwareMap.get(MagneticSensor.class, "magSensor");


        waitForStart();


        while (opModeIsActive()) {
            isFreightDetected = detectFreight(collector);
            setRobotMode();
            setCollector();
            setSlider(slider, lift);
            setConfig(lift);
            setLiftPosition(slider);
            setLift(lift);
            setDeliveryPosition();
            setLights(light);
            setDuckRoller(duck);
            setClaw(claw);

            if (intakeOn()) {
                collector.intakeSpeed(0.99);
            } else {
                collector.intakeSpeed(0);
            }

            if (ejectorOn()) {
                collector.egressSpeed(0.99);
                collector.intakeSpeed(0.99);
            } else {
                collector.egressSpeed(0);
                if (!intakeOn()) {
                    collector.intakeSpeed(0);
                }
            }

//            lightValue = gamepad1.left_trigger;
//            light.setPattern(lightValue);
//            telemetry.addData("lightValue: ", lightValue);


//            if (gamepad1.left_bumper) {
//                duck.spinLeft();
//            } else if (gamepad1.right_bumper) {
//                duck.spinRight();
//            } else {
//                duck.spinStop();
//            }
//
//            if (gamepad1.dpad_left) {
//                claw.extend();
//            } else if (gamepad1.dpad_right) {
//                claw.retract();
//            }

//            if (gamepad1.left_trigger > 0.5) {
//                claw.open();
//            } else if (gamepad1.right_trigger > 0.5) {
//                claw.close();
//            }

//            if (gamepad1.dpad_up) {
//                slider.speed(0.99);
//            } else if (gamepad1.dpad_down) {
//                slider.speed((-0.99));
//            } else {
//                slider.speed(0);
//            }


//            if (gamepad1.a) {
//                collector.intakeSpeed(0.99);
//            } else if (gamepad1.b) {
//                collector.egressSpeed(0.99);
//            } else if (gamepad1.x) {
//                collector.intakeSpeed(0);
//                collector.egressSpeed(0);
//            }

            leftStickX = gamepad1.left_stick_x;
            leftStickY = gamepad1.left_stick_y * -1;

            if (Math.abs(gamepad1.right_stick_x) > threshold) {
                if (gamepad1.right_stick_x < 0)
                    rightStickX = gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0);
                else
                    rightStickX = gamepad1.right_stick_x * gamepad1.right_stick_x * (4.0 / 5.0) + (1.0 / 5.0);
            } else
                rightStickX = 0;

                if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
                    //Calculate formula for mecanum drive function
                    double addValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 100;
                    double subtractValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 100;


                    //Set motor speed variables
                    if (robotMode == DELIVER) {
                        robot.setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
                    } else {
                        robot.setMotorPowers(-(addValue - rightStickX), -(subtractValue + rightStickX), -(subtractValue - rightStickX), -(addValue + rightStickX));
                    }
                } else {
                    robot.stop();
                }

//            showLeftFrontColors(slider);
//            showRightFrontColors(slider);
            showLiftEncoders(lift, slider);
            telemetry.update();

        }
    }

    private void setClaw(Claw claw) {
        if (!deployClawPressed) {
            if (gamepad1.left_bumper) {
                deployDuckRoller = false;
                deployClaw = !deployClaw;
                deployClawPressed = true;
            }
        } else {
            deployClawPressed = false;
        }

        if (deployClaw) {
            sliderOut = false;
            claw.extend();
            if (!clawOpenPressed && gamepad1.right_trigger > 0.5) {
                clawOpen = !clawOpen;
                clawOpenPressed = true;
            } else {
                clawOpenPressed = false;
            }

            if (!clawLiftUpPressed && gamepad1.left_trigger > 0.5) {
                clawLiftUp = !clawLiftUp;
                clawLiftUpPressed = true;
            } else {
                clawLiftUpPressed = false;
            }

            if (clawOpen) {
                claw.open();
            } else {
                claw.close();
            }
        } else {
            claw.retract();
        }
    }

    private void setDuckRoller(DuckRoller duck) {
        if (!deployDuckRollerPressed) {
            if (gamepad1.right_bumper) {
                deployClaw = false;
                deployDuckRoller = !deployDuckRoller;
                deployDuckRollerPressed = true;
            }
        } else {
            deployDuckRollerPressed = false;
        }

        if (deployDuckRoller) {
            sliderOut = false;
            duck.down();
            if (gamepad1.right_trigger > 0.5) {
                duck.spinRight();
            } else {
                duck.spinStop();
            }
        } else {
            duck.up();
        }
    }

    private void setLights(Light light) {
        if (robotMode == DELIVER) {
            if (liftSetting == LEVEL_1) {
                light.displayPattern(PURPLE_PATTERN);
            } else if (liftSetting == LEVEL_2) {
                light.displayPattern(GOLD_PATTERN);
            }  else if (liftSetting == LEVEL_3) {
                light.displayPattern(BLUE_PATTERN);
            } else {
                light.displayPattern(GREEN_PATTERN);
            }
        } else {
            light.displayPattern(RED_PATTERN);
        }
    }

    private void setDeliveryPosition() {
        if (!deliveryPositionPressed) {
            deliveryPositionPressed = true;
            if (gamepad1.dpad_up) {
                deliveryPositionCounter += 1;
            } else if (gamepad1.dpad_down) {
                deliveryPositionCounter = deliveryPositionCounter - 1;
            }

            if (deliveryPositionCounter > 2) {
                deliveryPositionCounter = 0;
            } else if (deliveryPositionCounter < 0) {
                deliveryPositionCounter = 2;
            }
        } else {
            deliveryPositionPressed = false;
        }

        if (!deployDuckRoller && !deployClaw) {
            switch (deliveryPositionCounter) {
                case 0:
                    deliveryPosition = LEVEL_3;
                    break;
                case 1:
                    deliveryPosition = LEVEL_1;
                    break;
                case 2:
                    deliveryPosition = LEVEL_2;
                    break;
            }
        }
    }

    private void setRobotMode() {
        if (gamepad1.y && robotMode == COLLECT) {
            if(!robotModePressed){
                robotMode = DELIVER;
                robotModePressed = true;
                sliderOut = false;
            }
        } else if (gamepad1.y && robotMode == DELIVER) {
            if(!robotModePressed){
                robotMode = COLLECT;
                robotModePressed = true;
                sliderOut = false;
            }
        } else {
            robotModePressed = false;
        }
    }

    private void setLiftPosition(Slider slider) {
        if (deployDuckRoller) {
            sliderForward = false;
            sliderBack = false;
            collectorOn = false;
            clawOpen =false;
            if (!slider.isForward() && !slider.isCenterForward() &&
                    !slider.isBack() && !slider.isCenterBack()) {
                if (gamepad1.right_trigger > 0.5) {
                    liftSetting = DUCK_SPIN;
                } else {
                    liftSetting = DUCK_APPROACH;
                }
            }
        } else if (deployClaw) {
            sliderBack = false;
            collectorOn = false;
            sliderForward = false;
            if (!slider.isForward() && !slider.isCenterForward() &&
                    !slider.isBack() && !slider.isCenterBack()) {
                if (clawLiftUp) {
                    liftSetting = CAP;
                } else {
                    liftSetting = CAP_PICKUP;
                }
            }
        } else {
            clawOpen =true;
            if (robotMode == COLLECT && sliderOut) {
                if (!slider.isForward() && !slider.isCenterForward()) {
                    liftSetting = COLLECTION;
                }
                sliderForward = false;
                sliderBack = true;
                collectorOn = true;
            } else if (robotMode == COLLECT && !sliderOut) {
                if (slider.isCenter()) {
                    liftSetting = MOVE;
                }
                sliderForward = false;
                sliderBack = false;
                collectorOn = false;
            } else if (robotMode == DELIVER && sliderOut) {
                if (!slider.isBack() && !slider.isCenterBack()) {
                    liftSetting = deliveryPosition;
                }
                sliderForward = true;
                sliderBack = false;
                collectorOn = false;
            } else if (robotMode == DELIVER && !sliderOut) {
                if (slider.isCenter()) {
                    liftSetting = MOVE;
                }
                sliderForward = false;
                sliderBack = false;
                collectorOn = false;
            }
        }
    }

    private void setLift(Lift lift) {
        switch (liftSetting) {
            case COLLECTION:
                liftPosition = COLLECTION_POSITION;
                break;
            case LEVEL_1:
                liftPosition = DELIVERY_1_POSITION;
                break;
            case LEVEL_2:
                liftPosition = DELIVERY_2_POSITION;
                break;
            case LEVEL_3:
                liftPosition = DELIVERY_3_POSITION;
                break;
            case CAP:
                liftPosition = CAPPING_POSITION;
                break;
            case PARK:
                liftPosition = PARK_POSITION;
                break;
            case MOVE:
                liftPosition = MOVE_POSITION;
                break;
            case DUCK_APPROACH:
                liftPosition = DUCK_APPROACH_POSITION;
                break;
            case DUCK_SPIN:
                liftPosition = DUCK_SPIN_POSITION;
                break;
            case CAP_PICKUP:
                liftPosition = CAP_PICKUP_POSITION;
                break;
        }
        lift.runToPosition(liftPosition);
    }

    private void setConfig(Lift lift) {
        switch (lift.liftPosition()) {
            case COLLECTION:
                sliderBackEnabled = true;
                sliderForwardEnabled = false;
                break;
            case DELIVERY:
                sliderBackEnabled = false;
                sliderForwardEnabled = true;
                break;
            case PARK:
                sliderBackEnabled = false;
                sliderForwardEnabled = false;
                break;
            case MOVE:
                sliderBackEnabled = false;
                sliderForwardEnabled = false;
                break;
            case TRANSIT:
                sliderBackEnabled = false;
                sliderForwardEnabled = false;
                break;
        }
    }

    private void setCollector() {
        if (gamepad1.right_trigger > 0.5) {
            if(!collectorButtonPressed){
                collectorOn = !collectorOn;
                collectorButtonPressed = true;
            }
        } else{
            collectorButtonPressed = false;
        }
    }

    private void setSlider(Slider slider, Lift lift) {
        if (gamepad1.a) {
            if(!sliderPressed){
                sliderOut = !sliderOut;
                sliderPressed = true;
            }
        } else{
            sliderPressed = false;
        }

        if (sliderForward && sliderForwardEnabled) {
            slider.forward(lift);
        } else if (sliderBack && sliderBackEnabled) {
            slider.back(lift);
        } else {
            slider.center(lift);
        }
    }

    private boolean moveSliderForward() {
        if (!isForwardStopDetected && sliderForward && sliderForwardEnabled) {
            return true;
        } else {
            return false;
        }
    }

    private boolean moveSliderBack() {
        if (!isBackStopDetected && sliderBack && sliderBackEnabled) {
            return true;
        } else {
            return false;
        }
    }

    private boolean intakeOn() {
        if (!isFreightDetected && collectorOn) {
            return true;
        } else {
            if (isFreightDetected && !collectorButtonPressed) {
                collectorOn = false;
                if (robotMode == COLLECT) {
                    sliderOut = false;
                }
                robotMode = DELIVER;
            }
            return false;
        }
    }

    private boolean ejectorOn() {
        if (gamepad1.right_trigger > 0.5 && !deployDuckRoller && !deployClaw) {
            return true;
        } else {
            return false;
        }
    }

    private boolean detectFreight(Collector collector) {
        NormalizedRGBA colors = collector.getFreightColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] > 10) {
            return true;
        } else {
            return false;
        }
    }

    public void showFreightColors(Collector collector) {
        NormalizedRGBA colors = collector.getFreightColor();
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
        telemetry.addData("Distance (cm)", "%.3f", collector.getFreightDistance());
    }

    public void showLeftFrontColors(Slider slider) {
        NormalizedRGBA colors = slider.getLeftColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
//        telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.addData("Distance (cm)", "%.3f", slider.getLeftDistance());
    }

    public void showRightFrontColors(Slider slider) {
        NormalizedRGBA colors = slider.getRightColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
//        telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.addData("Distance (cm)", "%.3f", slider.getRightDistance());
    }

    public void showMagData(MagneticSensor magSensor) {
        //            telemetry.addData("Manufacturer", magSensor.getManufacturerIDRaw());
//            telemetry.addData("Data 0 Low", magSensor.getMag0LowRaw());
//            telemetry.addData("Data 0 High", magSensor.getMag0HighRaw());
//            telemetry.addData("Data 1 Low", magSensor.getMag1LowRaw());
//            telemetry.addData("Data 1 High", magSensor.getMag1HighRaw());
//            telemetry.addData("Data 2 Low", magSensor.getMag2LowRaw());
//            telemetry.addData("Data 2 High", magSensor.getMag2HighRaw());
//            telemetry.addData("Data 3 Low", magSensor.getMag3LowRaw());
//            telemetry.addData("Data 3 High", magSensor.getMag3HighRaw());
    }

    public void showLiftEncoders(Lift lift, Slider slider) {
        telemetry.addData("Robot Mode: ", robotMode);
//        telemetry.addData("Left Lift: ", lift.leftLiftPosition());
//        telemetry.addData("Right Lift: ", lift.rightLiftPosition());
        telemetry.addData("Lift State: ", lift.liftPosition());
        telemetry.addData("Lift Setting: ", liftSetting);
//        telemetry.addData("Center Forward", slider.isCenterForward());
//        telemetry.addData("Center Back", slider.isCenterBack());
//        telemetry.addData("Center", slider.isCenter());
//        telemetry.addData("Forward", slider.isForward());
//        telemetry.addData("Back", slider.isBack());
//        telemetry.addData("Slider Forward: ", sliderForward);
//        telemetry.addData("Slider Forward Enabled: ", sliderForwardEnabled);
//        telemetry.addData("Slider Forward Stop: ", slider.isForward());
//        telemetry.addData("Slider Back: ", sliderBack);
//        telemetry.addData("Slider Back Enabled: ", sliderBackEnabled);
//        telemetry.addData("Slider Back Stop: ", slider.isBack());
//        telemetry.addData("Slider Lift Back Clear: ", lift.isBackClear());
//        telemetry.addData("Center Detected: ", slider.isCenter());
    }
}