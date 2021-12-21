package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Claw
{
    private Servo claw;
    private Servo clawShoulder;
    private Servo clawElbow;

    private OpMode opMode;


    /* Constructor */
    public Claw(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        claw = hwMap.servo.get("claw");
        clawShoulder = hwMap.servo.get("clawShoulder");
        clawElbow = hwMap.servo.get("clawElbow");

    }

    public void elbowUp() {
        clawElbow.setPosition(0.01);
    }

    public void elbowDown() {
        clawElbow.setPosition(0.99);
    }

    public void shoulderUp() {
        clawShoulder.setPosition(0.01);
    }

    public void shoulderDown() {
        clawShoulder.setPosition(0.5);
    }

    public void open() {
        claw.setPosition(0.75);
    }

    public void close() {
        claw.setPosition(0.2);
    }

    public void extend() {
        elbowDown();
        shoulderDown();
    }

    public void retract() {
        shoulderUp();
        elbowUp();
    }





}