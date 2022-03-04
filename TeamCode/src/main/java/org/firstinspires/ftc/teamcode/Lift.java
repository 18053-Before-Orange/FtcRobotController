package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class Lift
{
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor.RunMode currentLiftMode;
    public DigitalChannel magnetic;


    private OpMode opMode;

    public static final String COLLECTION = "COLLECTION";
    public static final String DELIVERY = "DELIVERY";
    public static final String PARK = "PARK";
    public static final String MOVE = "MOVE";
    public static final String TRANSIT = "TRANSIT";
    public static final String LEVEL_1 = "LEVEL 1";
    public static final String LEVEL_2 = "LEVEL 2";
    public static final String LEVEL_3 = "LEVEL 3";
    public static final String CAP = "CAP";
    public static final String CAP_PLACE = "CAP PLACE";
    public static final String CAP_PICKUP = "CAP PICKUP";
    public static final String DUCK_APPROACH = "DUCK APPROACH";
    public static final String DUCK_SPIN = "DUCK SPIN";

    public static final int COLLECTION_POSITION = 50;
    public static final int DELIVERY_1_POSITION = 550;
    public static final int DELIVERY_2_POSITION = 1250;
    public static final int DELIVERY_3_POSITION = 2150;
    public static final int CAPPING_POSITION = 3100;
    public static final int CAP_PLACEMENT_POSITION = 2300;
    public static final int CAP_PICKUP_POSITION = 200;
    public static final int PARK_POSITION = 0;
    public static final int MOVE_POSITION = 475;
    public static final int DUCK_APPROACH_POSITION = 250;
    public static final int DUCK_SPIN_POSITION = 250;


    /* Constructor */
    public Lift(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        leftLift = hwMap.dcMotor.get("leftLift");
        rightLift = hwMap.dcMotor.get("rightLift");

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        magnetic = hwMap.get(DigitalChannel.class, "magnetic");
        magnetic.setMode(DigitalChannel.Mode.INPUT);

    }

    public int runToSafetyPosition(int position, int holdPosition, SliderMotor slider) {
        if (slider.isCenter() || (slider.isForward() && position >= 450 && leftLiftPosition() > 520
                && (slider.getSliderPosition() < 60 || position > -holdPosition)) ||
                (slider.isBack() && position < 100 && leftLiftPosition() < 100)) {
            if (slider.getSliderPosition() >= 60 && position < -holdPosition) {
                leftLift.setTargetPosition(holdPosition);
                rightLift.setTargetPosition(holdPosition);

                slider.runToPosition(50);
            } else {
                leftLift.setTargetPosition(-position);
                rightLift.setTargetPosition(-position);
            }

            holdPosition = leftLift.getCurrentPosition();

        } else {
            leftLift.setTargetPosition(holdPosition);
            rightLift.setTargetPosition(holdPosition);
        }

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setPower(1.0);
        rightLift.setPower(1.0);

        return holdPosition;
    }


    public void runToPosition(int position) {

//        if (!magnetic.getState()) {
            leftLift.setTargetPosition(-position);
            rightLift.setTargetPosition(-position);

//        } else {
//            leftLift.setTargetPosition(leftLift.getCurrentPosition());
//            rightLift.setTargetPosition(rightLift.getCurrentPosition());
//        }

        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setPower(1.0);
        rightLift.setPower(1.0);

    }

    public boolean isForwardClear() {
        return liftPosition() == DELIVERY;
    }

    public boolean isBackClear() {
        return liftPosition() == COLLECTION;
    }

    public void resetEncoders() {
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public String liftPosition() {
        if (parkPosition()) {
            return PARK;
        } else if (collectPosition()) {
            return COLLECTION;
        } else if (movePosition()) {
            return MOVE;
        } else if (deliveryPosition()) {
            return DELIVERY;
        } else {
            return TRANSIT;
        }
    }

    public boolean parkPosition() {
        return (leftLiftPosition() < 30 && rightLiftPosition() < 30);
    }

    public boolean collectPosition() {
        return (leftLiftPosition() < 150);
    }

    public boolean movePosition() {
        return (leftLiftPosition() >= 400 && leftLiftPosition() < 500 &&
                rightLiftPosition() >= 400 && rightLiftPosition() < 500);
    }

    public boolean deliveryPosition() {
        return (leftLiftPosition() >= 520 && rightLiftPosition() >= 520);
    }

    public int leftLiftPosition() {
        return -leftLift.getCurrentPosition();
    }

    public int rightLiftPosition() {
        return -rightLift.getCurrentPosition();
    }

    public void liftUp() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setPower(1.0);
        rightLift.setPower(1.0);
    }

    public void liftDown() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setPower(-1.0);
        rightLift.setPower(-1.0);
    }

    public void liftStop() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setPower(0.0);
        rightLift.setPower(0.0);
    }
}