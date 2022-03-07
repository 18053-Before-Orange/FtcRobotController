package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class CollectorMotor
{

    private DcMotor collector;
    private CRServo ejectorLeft;
    private CRServo ejectorRight;
    private DistanceSensor distanceSensor;

    private OpMode opMode;

    NormalizedColorSensor freightDetector;

    float gain = 2;


    /* Constructor */
    public CollectorMotor(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        ejectorLeft = hwMap.crservo.get("ejectorLeft");
        ejectorRight = hwMap.crservo.get("ejectorRight");
        collector = hwMap.dcMotor.get("collector");
        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        freightDetector = hwMap.get(NormalizedColorSensor.class, "freightDetector");


        if (freightDetector instanceof SwitchableLight) {
            ((SwitchableLight) freightDetector).enableLight(true);
        }

        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean detectFreight() {
        NormalizedRGBA colors = getFreightColor(freightDetector);
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] > 10) {
            return true;
        } else {
            return false;
        }
    }

    public void egressSpeed(double power) {
        ejectorLeft.setPower(-power);
        ejectorRight.setPower(power);
    }

    public NormalizedRGBA getFreightColor(NormalizedColorSensor sensor) {
        return sensor.getNormalizedColors();
    }

    public double getFreightDistance(NormalizedColorSensor sensor) {
        return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
    }

    public void speed(double speed) {
        collector.setPower(speed);
    }
}