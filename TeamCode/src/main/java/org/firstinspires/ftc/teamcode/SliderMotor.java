package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;


public class SliderMotor
{
    private DcMotor slider;

    public static final int SLIDER_1_POSITION = 135;
    public static final int SLIDER_2_POSITION = 150;
    public static final int SLIDER_3_POSITION = 175;

//    NormalizedColorSensor sliderPosition;

    private OpMode opMode;


    /* Constructor */
    public SliderMotor(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        slider = hwMap.dcMotor.get("slider");
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        sliderPosition = hwMap.get(NormalizedColorSensor.class, "sliderPosition");
    }

    public void initializeEncoders(Lift lift) {
        int position = 1000;
        boolean reverse = false;
        lift.runToPosition(100);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (lift.touch.isPressed()) {
            if (position != slider.getCurrentPosition() && !reverse) {
                slider.setPower(0.25);
            } else if (reverse) {
                slider.setPower(-0.25);
            } else if (position == slider.getCurrentPosition()) {
                reverse = true;
            }
            position = slider.getCurrentPosition();
        }
        resetEncoders();
        lift.runToPosition(0);
    }

    public void resetEncoders() {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBack() {
        return getSliderPosition() < -4;
    }

    public boolean isForward() {
        return getSliderPosition() > 4;
    }

    public void forward(Lift lift, int sliderExtra) {
        if (lift.isForwardClear()) {
            if (slider.getCurrentPosition() < 25 && slider.getCurrentPosition() > 10) {
                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slider.setPower(1.0);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            if (lift.leftLiftPosition() > 1500) {
                runToPosition(SLIDER_3_POSITION);
            } else if (lift.leftLiftPosition() > 1000) {
                runToPosition(SLIDER_2_POSITION);
            } else {
                runToPosition(SLIDER_1_POSITION + sliderExtra);
            }
        }
    }

    public void back(Lift lift) {
        if (lift.isBackClear()) {
            if (slider.getCurrentPosition() > -50 && (lift.leftLiftPosition() < 150)) {
                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slider.setPower(-1.0);
                try {
                    Thread.sleep(150);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            runToPosition(-100);
        }
    }

    public void runToPositionNoProtection(int position) {
        slider.setTargetPosition(position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1.0);
    }

    public void runToPosition(int position) {
        slider.setTargetPosition(position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1.0);
    }

    public void speed(double power) {
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(power);
    }

    public int getSliderPosition() {
        return slider.getCurrentPosition();
    }

    public double getSliderPower() {
        return slider.getPower();
    }

    public boolean getSliderBusy() {
        return slider.isBusy();
    }

    public boolean isCenter() {
        return (getSliderPosition() < 30 && getSliderPosition() > -10);
    }

    public void center(Lift lift) {
       if (!lift.touch.isPressed()) {
           speed(-1.0);
       } else {
           speed(0);
       }
    }

//    public NormalizedRGBA getPositionColor() {
//        return sliderPosition.getNormalizedColors();
//    }
//
//    public double getPositionDistance() {
//        return ((DistanceSensor) sliderPosition).getDistance(DistanceUnit.CM);
//    }
}