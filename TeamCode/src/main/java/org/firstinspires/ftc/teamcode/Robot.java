package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Robot
{
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor.RunMode currentDrivetrainMode;

    private Telemetry telemetry;
    private OpMode opMode;


    /* Constructor */
    public Robot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.telemetry = telemetry;
        this.opMode = opMode;
        leftFront = hwMap.dcMotor.get("leftFront");
        rightFront = hwMap.dcMotor.get("rightFront");
        leftRear = hwMap.dcMotor.get("leftRear");
        rightRear = hwMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentDrivetrainMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        setMotorModes(currentDrivetrainMode);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }



    public int leftFrontPosition() {
        return leftFront.getCurrentPosition();
    }

    public int rightFrontPosition() {
        return rightFront.getCurrentPosition();
    }

    public int leftRearPosition() {
        return leftRear.getCurrentPosition();
    }

    public int rightRearPosition() {
        return rightRear.getCurrentPosition();
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        if(mode != currentDrivetrainMode) {
            leftFront.setMode(currentDrivetrainMode);
            rightFront.setMode(currentDrivetrainMode);
            leftRear.setMode(currentDrivetrainMode);
            rightRear.setMode(currentDrivetrainMode);
            currentDrivetrainMode = mode;
        }
    }

    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

}