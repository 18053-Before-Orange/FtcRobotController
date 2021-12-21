package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Slider
{
    private CRServo leftRearSlider;
    private CRServo rightFrontSlider;

    NormalizedColorSensor leftFrontDetector;
    NormalizedColorSensor rightRearDetector;

    private OpMode opMode;


    /* Constructor */
    public Slider(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        leftRearSlider = hwMap.crservo.get("leftRearSlider");
        rightFrontSlider = hwMap.crservo.get("rightFrontSlider");

        leftFrontDetector = hwMap.get(NormalizedColorSensor.class, "leftFrontCollector");
        rightRearDetector = hwMap.get(NormalizedColorSensor.class, "rightRearCollector");

    }

    public void speed(double power) {
        leftRearSlider.setPower(power);
        rightFrontSlider.setPower(-power);
    }

    public void forward(Lift lift) {
        if (!isForward() && lift.isForwardClear()) {
            speed(0.99);
        } else {
            speed(0);
        }
    }

    public void back(Lift lift) {
        if (!isBack() && lift.isBackClear()) {
            speed(-0.99);
        } else {
            speed(0);
        }
    }

    public boolean isCenter() {
        NormalizedRGBA leftColors = getLeftColor();
        final float[] leftHsvValues = new float[3];
        Color.colorToHSV(leftColors.toColor(), leftHsvValues);

        NormalizedRGBA rightColors = getRightColor();
        final float[] rightHsvValues = new float[3];
        Color.colorToHSV(rightColors.toColor(), rightHsvValues);

        if ((leftHsvValues[0] >= 200 && leftHsvValues[0] <= 240) ||
                (rightHsvValues[0] >= 200 && rightHsvValues[0] <= 240)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isCenterForward() {
        NormalizedRGBA colors = getRightColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[0] >= 0 && hsvValues[0] <= 40 && getRightDistance() < 3) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isCenterBack() {
        NormalizedRGBA colors = getLeftColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[0] >= 0 && hsvValues[0] <= 40 && getLeftDistance() < 3) {
            return true;
        } else {
            return false;
        }
    }

    public void center(Lift lift) {
        if (!isCenter()) {
            if (!isCenterForward() && !isForward()) {
                if (isBack()) {
                    speed(0.99);
                } else {
                    speed(0.35);
                }
            } else if (!isCenterBack() && !isBack()) {
                if (isForward()) {
                    speed(-0.99);
                } else {
                    speed(-0.35);
                }
            } else {
                speed(0);
            }
        } else {
            speed(0);
        }
    }

    public boolean detectCenterStop() {
        NormalizedRGBA leftColors = getLeftColor();
        NormalizedRGBA rightColors = getLeftColor();

        if ((leftColors.alpha > 0.2 && leftColors.alpha < 0.4 && getLeftDistance() < 3) &&
            (rightColors.alpha > 0.2 && rightColors.alpha < 0.4 && getRightDistance() < 3)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isForward() {
        NormalizedRGBA colors = getRightColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] >= 150 && hsvValues[0] <= 170  && getRightDistance() < 1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isBack() {
        NormalizedRGBA colors = getLeftColor();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] >= 150 && hsvValues[0] <= 170 && getLeftDistance() < 1) {
            return true;
        } else {
            return false;
        }
    }

    public NormalizedRGBA getLeftColor() {
        return leftFrontDetector.getNormalizedColors();
    }

    public double getLeftDistance() {
        return ((DistanceSensor) leftFrontDetector).getDistance(DistanceUnit.CM);
    }

    public NormalizedRGBA getRightColor() {
        return rightRearDetector.getNormalizedColors();
    }

    public double getRightDistance() {
        return ((DistanceSensor) rightRearDetector).getDistance(DistanceUnit.CM);
    }
}