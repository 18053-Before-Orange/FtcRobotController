package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Collector
{
    private CRServo leftFrontSpinner;
    private CRServo rightFrontSpinner;
    private CRServo leftRearSpinner;
    private CRServo rightRearSpinner;

    private OpMode opMode;

    NormalizedColorSensor freightDetector;
    float gain = 2;


    /* Constructor */
    public Collector(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        leftFrontSpinner = hwMap.crservo.get("leftFrontSpinner");
        rightFrontSpinner = hwMap.crservo.get("rightFrontSpinner");
        leftRearSpinner = hwMap.crservo.get("leftRearSpinner");
        rightRearSpinner = hwMap.crservo.get("rightRearSpinner");

        freightDetector = hwMap.get(NormalizedColorSensor.class, "freightDetector");

        if (freightDetector instanceof SwitchableLight) {
            ((SwitchableLight) freightDetector).enableLight(true);
        }

    }

    public NormalizedRGBA getFreightColor() {
        return freightDetector.getNormalizedColors();
    }

    public double getFreightDistance() {
        return ((DistanceSensor) freightDetector).getDistance(DistanceUnit.CM);
    }

    public void egressSpeed(double power) {
        leftFrontSpinner.setPower(power);
        rightFrontSpinner.setPower(-power);
    }

    public void intakeSpeed(double power) {
        leftRearSpinner.setPower(power);
        rightRearSpinner.setPower(-power);
    }
}