package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class DuckRoller
{
    private Servo duckShoulder;
    private CRServo duckRoller;

    private OpMode opMode;
    private double duckPower = 0.75;


    /* Constructor */
    public DuckRoller(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        duckShoulder = hwMap.servo.get("duckRollerShoulder");
        duckRoller = hwMap.crservo.get("duckRollerSpinner");

    }

    public void down() {
       duckShoulder.setPosition(0.1);
    }

    public void up() {
        duckShoulder.setPosition(0.7);
    }

    public void spinLeft() {
        duckRoller.setPower(duckPower);
    }

    public void spinRight() {
        duckRoller.setPower(-duckPower);
    }

    public void spinStop() {
        duckRoller.setPower(0);
    }


}