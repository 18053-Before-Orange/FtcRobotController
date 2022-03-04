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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static org.firstinspires.ftc.teamcode.Lift.CAP;
import static org.firstinspires.ftc.teamcode.Lift.CAPPING_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.CAP_PICKUP;
import static org.firstinspires.ftc.teamcode.Lift.CAP_PICKUP_POSITION;
import static org.firstinspires.ftc.teamcode.Lift.CAP_PLACE;
import static org.firstinspires.ftc.teamcode.Lift.CAP_PLACEMENT_POSITION;
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

@TeleOp(name="RedAlliance", group="18053")
public class RedAllianceDefault extends LinearOpMode {
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
    private boolean markerPlace = false;
    private boolean markerPlacePressed = false;
    private boolean levelChangePressed = false;
    private boolean sliderReset = false;
    private int holdPosition = 0;
    private int sliderExtra = 0;
    private int duckExtra = 0;
    private int capPlaceExtra = 0;
    private int capPickupExtra = 0;
    private int deliveryAdjustment = 0;

    private static final RevBlinkinLedDriver.BlinkinPattern GREEN_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
    private static final RevBlinkinLedDriver.BlinkinPattern RED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
    private static final RevBlinkinLedDriver.BlinkinPattern PURPLE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
    private static final RevBlinkinLedDriver.BlinkinPattern GOLD_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    private static final RevBlinkinLedDriver.BlinkinPattern BLUE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
    private static final RevBlinkinLedDriver.BlinkinPattern STROBE_BLUE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
    private static final RevBlinkinLedDriver.BlinkinPattern STROBE_RED_PATTERN = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
    private static final RevBlinkinLedDriver.BlinkinPattern STROBE_WHITE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
    private static final double threshold = 0.1;
    private static final String DELIVER = "DELIVER";
    private static final String COLLECT = "COLLECT";
    private String robotMode = COLLECT;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        Lift lift = new Lift(hardwareMap, telemetry, this);
        CollectorMotor collector = new CollectorMotor(hardwareMap, telemetry, this);
        SliderMotor slider = new SliderMotor(hardwareMap, telemetry, this);
        DuckRoller duck = new DuckRoller(hardwareMap, telemetry, this);
        Claw claw = new Claw(hardwareMap, telemetry, this);
        Light light = new Light(hardwareMap, telemetry, this);


        telemetry.addLine("Hit A to reset Slider Encoders, B to NOT reset Slider encoders");
        telemetry.update();

        if (!lift.magnetic.getState()) {
            light.displayPattern(STROBE_BLUE_PATTERN);
        } else {
            light.displayPattern(STROBE_RED_PATTERN);
        }

        while (!gamepad1.a && !gamepad1.b && !opModeIsActive() && !isStopRequested()) {

        }

        if (gamepad1.a) {
            slider.resetEncoders();
            telemetry.addLine("Slider Encoders reset");
        } else {
            telemetry.addLine("Slider Encoders NOT reset");
        }

        light.displayPattern(RED_PATTERN);

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.5) {
                if (!lift.magnetic.getState()) {
                    light.displayPattern(STROBE_BLUE_PATTERN);
                } else {
                    light.displayPattern(STROBE_RED_PATTERN);
                }

                if (gamepad1.start) {
                    slider.resetEncoders();
                }

                if (gamepad1.back) {
                    lift.resetEncoders();
                }

                if (gamepad1.dpad_down) {
                    lift.liftUp();
                } else if (gamepad1.dpad_up) {
                    lift.liftDown();
                } else {
                    lift.runToPosition(lift.leftLiftPosition());
                }

                if (gamepad1.dpad_left) {
                    slider.speed(0.25);
                } else if (gamepad1.dpad_right) {
                    slider.speed(-0.25);
                } else {
                    slider.speed(0);
                }
            } else {

                if (gamepad1.start) {
                    light.displayPattern(STROBE_WHITE_PATTERN);
                    if (gamepad1.dpad_up) {
                        deliveryAdjustment += 15;
                    } else if (gamepad1.dpad_down) {
                        deliveryAdjustment = deliveryAdjustment - 15;
                    }
                }

                isFreightDetected = collector.detectFreight();
                setRobotMode();
                setCollector();
                setSlider(slider, lift);
                setConfig(lift);
                setLiftPosition(slider, claw);
                setLift(lift, slider);
                setDeliveryPosition();
                setLights(light);
                setDuckRoller(duck);
                setClaw(claw);

                if (ejectorOn()) {
                    collector.speed(0.9);
                    collector.egressSpeed(0.99);
                    sleep(150);
                    collector.speed(0.0);
                    collector.egressSpeed(0.0);
                } else {
                    if (!intakeOn()) {
                        collector.speed(0);
                        collector.egressSpeed((0));
                    } else {
                        collector.speed(0.9);
                    }
                }

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
                    double addValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 100;
                    double subtractValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 100;


                    //Set motor speed variables
                    if (robotMode == DELIVER) {
                        robot.setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
                    } else {
                        robot.setMotorPowers(-(addValue - rightStickX), -(subtractValue + rightStickX), -(subtractValue - rightStickX), -(addValue + rightStickX));
                    }
                } else {
                    robot.stop();
                }

//                            showFreightColors(slider);
                showLiftEncoders(lift, slider, robot);
                telemetry.update();
            }

        }
    }

    private void setClaw(Claw claw) {
        if (gamepad1.left_bumper) {
            if (!deployClawPressed) {
                deployDuckRoller = false;
                deployClaw = !deployClaw;
                deployClawPressed = true;
            }
        } else {
            deployClawPressed = false;
        }

        if (deployClaw && !markerPlacePressed && gamepad1.a) {
            markerPlace = !markerPlace;
            markerPlacePressed = true;
        } else {
            markerPlacePressed = false;
        }

        if (deployClaw) {
            sliderOut = false;

            if (gamepad1.right_trigger > 0.5) {
                if (!clawOpenPressed) {
                    clawOpen = !clawOpen;
                    clawOpenPressed = true;
                }
            } else {
                clawOpenPressed = false;
            }

            if (gamepad1.b) {
                if (!clawLiftUpPressed) {
                    clawLiftUp = !clawLiftUp;
                    clawLiftUpPressed = true;
                }
            } else {
                clawLiftUpPressed = false;
            }

            if (clawOpen && !clawOpenPressed) {
                claw.open();
            } else {
                claw.close();
            }
        } else {
            claw.retract();
        }
    }

    private void setDuckRoller(DuckRoller duck) {
        if (gamepad1.right_bumper) {
            if (!deployDuckRollerPressed) {
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
                duck.spin(-0.99);
            } else {
                duck.spinStop();
            }
        } else {
            duck.up();
        }
    }

    private void setLights(Light light) {
        if (gamepad1.left_trigger < 0.5 && !gamepad1.start) {
            if (robotMode == COLLECT) {
                light.displayPattern(RED_PATTERN);
            } else if (liftSetting == LEVEL_1) {
                light.displayPattern(PURPLE_PATTERN);
            } else if (liftSetting == LEVEL_2) {
                light.displayPattern(GOLD_PATTERN);
            } else if (liftSetting == LEVEL_3) {
                light.displayPattern(BLUE_PATTERN);
            } else {
                light.displayPattern(GREEN_PATTERN);
            }
        }
    }

    private void setDeliveryPosition() {
        if (!deployDuckRoller && !deployClaw) {
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
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
                }
            } else {
                deliveryPositionPressed = false;
            }

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

    private void setLiftPosition(SliderMotor slider, Claw claw) {
        if (deployDuckRoller) {
            sliderForward = false;
            sliderBack = false;
            collectorOn = false;
            clawOpen = false;
            liftSetting = DUCK_SPIN;
            if (gamepad1.dpad_down) {
                duckExtra = duckExtra - 20;
            } else if (gamepad1.dpad_up) {
                duckExtra += 20;
            }

        } else if (deployClaw) {
            sliderBack = false;
            collectorOn = false;
            sliderForward = false;
            if (slider.isCenter()) {
                if (clawLiftUp) {
                    liftSetting = CAP;
                    claw.place();
                    if (markerPlace) {
                        if (gamepad1.dpad_up) {
                            capPlaceExtra += 20;
                        } else if (gamepad1.dpad_down) {
                            capPlaceExtra = capPlaceExtra - 20;
                        }
                        liftSetting = CAP_PLACE;
                    } else {
                        liftSetting = CAP;
                    }
                } else {
                    if (gamepad1.dpad_up) {
                        capPickupExtra += 20;
                    } else if (gamepad1.dpad_down) {
                        capPickupExtra = capPickupExtra - 20;
                    }
                    liftSetting = CAP_PICKUP;
                    claw.extend();
                }
            }
        } else {
            clawLiftUp = false;
            clawOpen = false;
            markerPlace = false;
            claw.close();
            if (robotMode == COLLECT && sliderOut) {
                liftSetting = COLLECTION;
                sliderForward = false;
                sliderBack = true;
                collectorOn = true;
            } else if (robotMode == COLLECT && !sliderOut) {
                liftSetting = MOVE;
                sliderForward = false;
                sliderBack = false;
                collectorOn = false;
            } else if (robotMode == DELIVER && sliderOut) {
                liftSetting = deliveryPosition;
                sliderForward = true;
                sliderBack = false;
                collectorOn = false;
            } else if (robotMode == DELIVER && !sliderOut) {
                liftSetting = MOVE;
                sliderForward = false;
                sliderBack = false;
                collectorOn = false;
            }
        }
    }

    private void setLift(Lift lift, SliderMotor slider) {
        switch (liftSetting) {
            case COLLECTION:
                liftPosition = COLLECTION_POSITION;
                holdPosition = lift.runToSafetyPosition(liftPosition, holdPosition, slider);
                break;
            case LEVEL_1:
                liftPosition = DELIVERY_1_POSITION + deliveryAdjustment;
                holdPosition = lift.runToSafetyPosition(liftPosition, holdPosition, slider);
                break;
            case LEVEL_2:
                liftPosition = DELIVERY_2_POSITION + deliveryAdjustment;
                holdPosition = lift.runToSafetyPosition(liftPosition, holdPosition, slider);
                break;
            case LEVEL_3:
                liftPosition = DELIVERY_3_POSITION + deliveryAdjustment;
                holdPosition = lift.runToSafetyPosition(liftPosition, holdPosition, slider);
                break;
            case CAP:
                liftPosition = CAPPING_POSITION;
                holdPosition = liftPosition;
                lift.runToPosition(liftPosition);
                break;
            case PARK:
                liftPosition = PARK_POSITION;
                holdPosition = lift.runToSafetyPosition(liftPosition, holdPosition, slider);
                break;
            case MOVE:
                liftPosition = MOVE_POSITION;
                holdPosition = lift.runToSafetyPosition(liftPosition, holdPosition, slider);
                break;
            case DUCK_APPROACH:
                liftPosition = DUCK_APPROACH_POSITION + duckExtra;
                holdPosition = liftPosition;
                lift.runToPosition(liftPosition);
                break;
            case DUCK_SPIN:
                liftPosition = DUCK_SPIN_POSITION + duckExtra;
                holdPosition = liftPosition;
                lift.runToPosition(liftPosition);
                break;
            case CAP_PICKUP:
                liftPosition = CAP_PICKUP_POSITION + capPickupExtra;
                holdPosition = liftPosition;
                lift.runToPosition(liftPosition);
                break;
            case CAP_PLACE:
                liftPosition = CAP_PLACEMENT_POSITION + capPlaceExtra;
                holdPosition = liftPosition;
                lift.runToPosition(liftPosition);
                break;
        }

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

    private void setSlider(SliderMotor slider, Lift lift) {
        if (!deployClaw) {
            if (gamepad1.b) {
                sliderExtra = sliderExtra - 15;
            } else if (gamepad1.x) {
                sliderExtra += 15;
            }
        }

        if (gamepad1.a) {
            if(!sliderPressed){
                sliderOut = !sliderOut;
                sliderPressed = true;
            }
        } else{
            sliderPressed = false;
        }

        if (!lift.magnetic.getState() && !sliderReset) {
//            slider.resetEncoders();
            sliderReset = true;
        }
        
        if (sliderForward && sliderForwardEnabled) {
            slider.forward(lift, sliderExtra);
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

//    public void showFreightColors(SliderMotor slider) {
//        NormalizedRGBA colors = slider.getPositionColor();
//        final float[] hsvValues = new float[3];
//        Color.colorToHSV(colors.toColor(), hsvValues);
////        telemetry.addLine()
////                .addData("Red", "%.3f", colors.red)
////                .addData("Green", "%.3f", colors.green)
////                .addData("Blue", "%.3f", colors.blue);
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
//        telemetry.addData("Distance (cm)", "%.3f", slider.getPositionDistance());
//        telemetry.addData("Slider", slider.getSliderPosition());
//    }

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

    public void showLiftEncoders(Lift lift, SliderMotor slider, Robot robot) {
        telemetry.addData("Robot Mode: ", robotMode);
        telemetry.addData("Left Lift: ", lift.leftLiftPosition());
//        telemetry.addData("Right Lift: ", lift.rightLiftPosition());
//        telemetry.addData("Claw: ", deployClaw);
//        telemetry.addData("Claw Pressed: ", deployClawPressed);
//        telemetry.addData("Claw Lift Up: ", clawLiftUp);
//        telemetry.addData("Lift State: ", lift.liftPosition());
        telemetry.addData("Lift Setting: ", liftSetting);
        telemetry.addData("Lift Position: ", liftPosition);
        telemetry.addData("Hold Position: ", holdPosition);
//        telemetry.addData("Magnet: ", lift.magnetic.getState());
        telemetry.addData("Slider Center: ", slider.isCenter());
        telemetry.addData("Slider Position: ", slider.getSliderPosition());
        telemetry.addData("Slider Power: ", slider.getSliderPower());
        telemetry.addData("Slider Busy: ", slider.getSliderBusy());
        telemetry.addData("Center", slider.isCenter());
        telemetry.addData("Forward", slider.isForward());
        telemetry.addData("Back", slider.isBack());
        telemetry.addData("Slider Forward: ", sliderForward);
        telemetry.addData("Slider Forward Enabled: ", sliderForwardEnabled);
        telemetry.addData("Slider Forward Stop: ", slider.isForward());
        telemetry.addData("Slider Back: ", sliderBack);
        telemetry.addData("Slider Back Enabled: ", sliderBackEnabled);
        telemetry.addData("Slider Back Stop: ", slider.isBack());
        telemetry.addData("Slider Lift Back Clear: ", lift.isBackClear());
        telemetry.addData("Center Detected: ", slider.isCenter());
    }
}