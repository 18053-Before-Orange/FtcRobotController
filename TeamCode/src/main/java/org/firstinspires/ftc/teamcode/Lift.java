package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift
{
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor.RunMode currentLiftMode;

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
    public static final String CAP_PICKUP = "CAP PICKUP";
    public static final String DUCK_APPROACH = "DUCK APPROACH";
    public static final String DUCK_SPIN = "DUCK SPIN";

    public static final int COLLECTION_POSITION = 75;
    public static final int DELIVERY_1_POSITION = 700;
    public static final int DELIVERY_2_POSITION = 1600;
    public static final int DELIVERY_3_POSITION = 2500;
    public static final int CAPPING_POSITION = 3200;
    public static final int CAP_PICKUP_POSITION = 100;
    public static final int PARK_POSITION = 0;
    public static final int MOVE_POSITION = 450;
    public static final int DUCK_APPROACH_POSITION = 150;
    public static final int DUCK_SPIN_POSITION = 450;


    /* Constructor */
    public Lift(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.opMode = opMode;
        leftLift = hwMap.dcMotor.get("leftLift");
        rightLift = hwMap.dcMotor.get("rightLift");

        rightLift.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void runToPosition(int position) {
        leftLift.setTargetPosition(-position);
        rightLift.setTargetPosition(-position);

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
        return (leftLiftPosition() < 50 && rightLiftPosition() < 50);
    }

    public boolean collectPosition() {
        return (leftLiftPosition() >= 50 && leftLiftPosition() < 100 &&
                rightLiftPosition() >= 50 && rightLiftPosition() < 100);
    }

    public boolean movePosition() {
        return (leftLiftPosition() >= 400 && leftLiftPosition() < 500 &&
                rightLiftPosition() >= 400 && rightLiftPosition() < 500);
    }

    public boolean deliveryPosition() {
        return (leftLiftPosition() >= 575 && rightLiftPosition() >= 575);
    }

    public int leftLiftPosition() {
        return -leftLift.getCurrentPosition();
    }

    public int rightLiftPosition() {
        return -rightLift.getCurrentPosition();
    }

}