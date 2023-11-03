package org.firstinspires.ftc.teamcode.G3GSS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.slf4j.Logger;

import java.util.List;

public class TESTG3S {

    //Declare motors
    public DcMotor left1 = null;
    public DcMotor left2 = null;
    public DcMotor right1 = null;
    public DcMotor right2 = null;
    public BNO055IMU imu = null;
    public Servo hook = null;

    double gyrotarget = 0;


    LinearOpMode runningOpMode;
    Logger logger;

    final int RIGHT_ENCODER_FACTOR = -1;
    final int LEFT_ENCODER_FACTOR = -1;
    final int ENCODER_THRESHOLD = 200;

    public TESTG3S(LinearOpMode RunningOpMode) {
        this.runningOpMode = RunningOpMode;
    }

    public void init (HardwareMap hwMap){
        init(hwMap, true);
    }


    //Declaring hardware map
    public void init(HardwareMap hwMap, boolean resetslidepause) {


        left1 = hwMap.dcMotor.get("left1");
        right1 = hwMap.dcMotor.get("right1");
        right2 = hwMap.dcMotor.get("right2");
        left2 = hwMap.dcMotor.get("left2");
        hook = hwMap.servo.get("hook");

        left1.setDirection(DcMotor.Direction.REVERSE);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.FORWARD);


        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        left1.setPower(0);
        right1.setPower(0);
        left2.setPower(0);
        right2.setPower(0);

        hook.scaleRange(0, 1);
        hook.setPosition(.6);


        //gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }



    public void driveStraight(double speedLeft, double speedRight, int target, double gyroTarget) throws InterruptedException {

        driveStraight(speedLeft, speedRight, target, gyroTarget, Integer.MAX_VALUE);

    }

    public void driveStraight(double speedLeft, double speedRight, int target, double gyroTarget, int Timeout) throws InterruptedException {

        /* Function for driving forward with encoders */

        runningOpMode.telemetry.addData("Entering", "driveSTraight");
        runningOpMode.telemetry.update();

//        logger.write("driveStraight");

        //get initial positions for encoders
        double currentPositionLeft = LEFT_ENCODER_FACTOR * left1.getCurrentPosition();
        double currentPositionRight = RIGHT_ENCODER_FACTOR * right1.getCurrentPosition();

        //set the new targets by adding them to the current position
        double newTargetLeft = target + currentPositionLeft;
        double newTargetRight = target + currentPositionRight;

        boolean TargetNegative = newTargetRight < currentPositionRight;

        ElapsedTime Timer = new ElapsedTime();
        Timer.reset();

        double lastEncoderReading = currentPositionRight;
        int numReads = 1;

        //loop will run while the encoder values are within 15 counts of their target & the opmode is active
        while (runningOpMode.opModeIsActive() && Math.abs(newTargetLeft - currentPositionLeft) > 200
                && Math.abs(newTargetRight - currentPositionRight) > 200 && evaldistance(currentPositionRight, newTargetRight, TargetNegative, 200)
                && Timer.milliseconds() < Timeout) {

            //get the heading
            double drift = getHeading();

            //set the speed of the motors according to the drift from the gyro target
            double leftSpeed = (speedLeft + (drift - gyroTarget) / 100);
            double rightSpeed = (speedRight - (drift - gyroTarget) / 100);


            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            left2.setPower(leftSpeed);
            right1.setPower(rightSpeed); //no encoders
            left1.setPower(leftSpeed); //no encoders
            right2.setPower(rightSpeed);

            currentPositionLeft = LEFT_ENCODER_FACTOR * left1.getCurrentPosition();
            currentPositionRight = RIGHT_ENCODER_FACTOR * right1.getCurrentPosition();

            lastEncoderReading = currentPositionRight;
            numReads++;

            runningOpMode.telemetry.addData("Status", "Busy");
            runningOpMode.telemetry.addData("currentPosRight", currentPositionRight);
            runningOpMode.telemetry.addData("currentPosLeft", currentPositionLeft);
            runningOpMode.telemetry.addData("heading", getHeading());
            runningOpMode.telemetry.addData("currentPowerl", leftSpeed);
            runningOpMode.telemetry.addData("currentPowerr", rightSpeed);
            runningOpMode.telemetry.update();
        }
        runningOpMode.idle();

        left2.setPower(0);
        right1.setPower(0);
        left1.setPower(0);
        right2.setPower(0);

        runningOpMode.telemetry.addData("Status", "Complete");
        runningOpMode.telemetry.update();

    }

    public void strafeWithSeconds(boolean strafeLeft, double leftspeed, double rightspeed, int sleep) {

        if (strafeLeft) {

            left1.setPower(-leftspeed);
            left2.setPower(leftspeed);
            right1.setPower(rightspeed);
            right2.setPower(-rightspeed);
        } else {
            left1.setPower(leftspeed);
            left2.setPower(-leftspeed);
            right1.setPower(-rightspeed);
            right2.setPower(rightspeed);
        }

        runningOpMode.sleep(sleep);
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);

        runningOpMode.telemetry.addData("Exiting", "Strafe with Seconds");
        runningOpMode.telemetry.update();

    }

    public void setdrivetrainbehavior(DcMotor.ZeroPowerBehavior floatbehavior) {
        left1.setZeroPowerBehavior(floatbehavior);
        right1.setZeroPowerBehavior(floatbehavior);
        left2.setZeroPowerBehavior(floatbehavior);
        right2.setZeroPowerBehavior(floatbehavior);

    }


    public void driveStraightWithSeconds(double leftSpeed, double rightSpeed, int sleep) {
        left1.setPower(leftSpeed);
        left2.setPower(leftSpeed);
        right1.setPower(rightSpeed);
        right2.setPower(rightSpeed);
        runningOpMode.sleep(sleep);
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);

    }

    public double getHeading() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = angles.firstAngle;

        return heading;
    }

    public static boolean evaldistance(double currentpos, double target, boolean TargetSmall, int threshold) {

        if (TargetSmall) {
            return currentpos - target > threshold;
        } else {
            return currentpos - target < threshold;

        }
    }




}
