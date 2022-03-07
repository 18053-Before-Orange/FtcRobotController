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
    private double duckPower = 0.3;


    /* Constructor */
    public DuckRoller(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        duckShoulder = hwMap.servo.get("duckShoulder");
        duckRoller = hwMap.crservo.get("duckRoller");

    }

    public void down() {
       duckShoulder.setPosition(0.09);
    }

    public void up() {
        duckShoulder.setPosition(0.7);
    }

    public void spinLeft() {
        duckRoller.setPower(-duckPower);
    }

    public void spinRight() {
        duckRoller.setPower(duckPower);
    }

    public void spinStop() {
        duckRoller.setPower(0);
    }

    public void spin(double power) {
        duckRoller.setPower(power);
    }

    public void deliverBlue() {
        duckRoller.setPower(0.99);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        duckRoller.setPower(0);
    }


}