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
import org.firstinspires.ftc.teamcode.G3GSS.Logger;

import java.util.List;

public class G3GSS {

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    //Declare cone labels
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private TFObjectDetector tfod;

    //Declare motors
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftslide = null;
    public DcMotor rightslide = null;
    public DcMotor arm = null;
    public Servo claw = null;
    public BNO055IMU imu = null;
    public DigitalChannel magswitch = null;
    public static ColorSensor colordetection = null;
    public DigitalChannel armstop = null;

    public G3GSS() {

    }

    //Declaring park zones
    public enum ParkZone {
        LeftZone,
        CenterZone,
        RightZone,
    }

//    public enum Zones {
//        GROUND_LEVEL,
//        SHORT_LEVEL,
//        MEDIUM_LEVEL,
//        TALL_LEVEL,
//    }

    //states
    public enum states {
        STATIONARY,
        START_MOVING,
        MOVING
    }
    //lift states
    
    //Strafe directions
        public enum StrafeDirection {
        Right, Left
    }


    states liftstates = states.STATIONARY;
    double Liftspeed = 0;
    double Lifttarget = 0;

    public final double LIFT_POS_GROUND_TARGET = 0;
    public final double LIFT_POS_LOW_TARGET = 1000;
    public final double LIFT_POS_MED_TARGET = 1500;
    public final double LIFT_POS_HIGH_TARGET = 2000;


    states armstates = states.STATIONARY;
    double armspeed = 0;
    double armtarget = 0;

    public final double ARM_POS_GROUND_TARGET = 0;
    public final double ARM_POS_LOW_TARGET = 100;
    public final double ARM_POS_MED_TARGET = 300;
    public final double ARM_POS_HIGH_TARGET = 500;



    double gyrotarget = 0;

    //Declaring Ultrasonic Sensors
    public UltrasonicSensor forwardleft;
    public UltrasonicSensor forwardright;
    public UltrasonicSensor backwardleft;
    public UltrasonicSensor backwardright;

    LinearOpMode runningOpMode;
    Logger logger;

    final int RIGHT_ENCODER_FACTOR = -1;
    final int LEFT_ENCODER_FACTOR = -1;
    final int ENCODER_THRESHOLD = 200;

    //Declaring Zones for the Terminal heights


    public G3GSS(LinearOpMode RunningOpMode, Logger logger) {
        this.runningOpMode = RunningOpMode;
        this.logger = logger;
    }

    public void init (HardwareMap hwMap){
        init(hwMap, true);
    }

    public void slidesmoveup(int target, double speed){
        leftslide.setTargetPosition(target);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightslide.setPower(speed);
        leftslide.setPower(Math.abs(speed));
        while (runningOpMode.opModeIsActive() && leftslide.isBusy()){
            runningOpMode.telemetry.addData("leftslide", leftslide.getCurrentPosition());
            runningOpMode.telemetry.update();

        }

        rightslide.setPower(0);


    }


    //Declaring hardware map
    public void init(HardwareMap hwMap, boolean resetslidepause) {




        leftFront = hwMap.dcMotor.get("leftFront");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightRear = hwMap.dcMotor.get("rightRear");
        leftRear = hwMap.dcMotor.get("leftRear");
        leftslide = hwMap.dcMotor.get("leftslide");
        rightslide = hwMap.dcMotor.get("rightslide");
        arm = hwMap.dcMotor.get("arm");
        claw = hwMap.servo.get("claw");
        colordetection = hwMap.get(ColorSensor.class, "colordetection");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftslide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightslide.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //right motor requires negative power
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if (resetslidepause){
            leftslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //claw position 0 is open
        claw.setPosition(0);
        arm.setPower(0);
        //arm.setPower(0.9);
        leftslide.setPower(0);
        rightslide.setPower(0);


        claw.scaleRange(0, 1);
        claw.setPosition(.6);


        magswitch = hwMap.digitalChannel.get("magswitch");
        magswitch.setMode(DigitalChannel.Mode.INPUT);
        armstop = hwMap.digitalChannel.get("armstop");
        armstop = hwMap.get(DigitalChannel.class, "armstop");
        armstop.setMode(DigitalChannel.Mode.INPUT);

        //gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }



    public void setliftState(states State, double speed, double target){

        liftstates = State;
        Lifttarget = target;
        Liftspeed = speed;

    }


    public void setarmStates(states State, double speed, double target){

        armstates = State;
        armtarget = target;
        armspeed = speed;

    }

    double newTargetlift = 0, currentPositionlift = 0;
    int speedModifier = 1;


    public void FSMGSS_SLIDES() {

        switch (liftstates) {

            case START_MOVING:

                leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //get initial positions for encoders
                currentPositionlift = leftslide.getCurrentPosition();

                //gets target value
                newTargetlift = Lifttarget;


                //gets modifier value
                if (currentPositionlift > newTargetlift) {
                    speedModifier = -1;
                } else if (currentPositionlift < newTargetlift) {
                    speedModifier = 1;
                } else {
                    speedModifier = 0;
                }

                double MotorSpeed = Liftspeed * speedModifier;

                liftstates = states.MOVING;
                break;

            case MOVING:

                if (runningOpMode.opModeIsActive() && Math.abs(newTargetlift - leftslide.getCurrentPosition()) > 300
                        && leftslide.getCurrentPosition() - currentPositionlift < 300) {


                    //clip the range of the motor powers to make sure it doesn't exceed 1
                    MotorSpeed = Range.clip(Liftspeed, -1, 1);

                    //Set power to preset Liftspeed
                    leftslide.setPower(MotorSpeed);
                    rightslide.setPower(MotorSpeed);

                    //Adding Data to the telemetry
                    runningOpMode.telemetry.addData("Status", "Busy");
                    runningOpMode.telemetry.addData("currentPosRight", leftslide.getCurrentPosition());
                    runningOpMode.telemetry.update();

                    liftstates = states.STATIONARY;
                }
                else {
                    liftstates = states.STATIONARY;
                }
                break;

            case STATIONARY:

                Lifttarget = 0;
                Liftspeed = 0;

                leftslide.setPower(0);
                rightslide.setPower(0);

            default:
                break;
        }
    }

    double newTargetRotate = 0, currentPositionArm = 0;
    int speedModifierArm = 1;

    //FSM for the arm
    public void G3GSSFSM_ARM() {

        switch (armstates) {

            case START_MOVING:

                runningOpMode.telemetry.addData("Entering", "Arm Rotations");
                runningOpMode.telemetry.update();

                //get initial position for encoders
                currentPositionArm = arm.getCurrentPosition();

                //set a new position we want to be at
                newTargetRotate = armtarget;

                //gets modifier value
                if (currentPositionlift > newTargetlift) {
                    speedModifierArm = -1;
                } else if (currentPositionlift < newTargetlift) {
                    speedModifierArm = 1;
                } else {
                    speedModifierArm = 0;
                }

                double MotorSpeed = armspeed * speedModifierArm;

                armstates = states.MOVING;

                break;

            case MOVING:

                runningOpMode.telemetry.addData("moving", "");
                runningOpMode.telemetry.update();

                if (runningOpMode.opModeIsActive() && Math.abs(newTargetRotate - currentPositionArm) > 60
                        && evaldistance(currentPositionArm, newTargetRotate, speedModifier < 0, 60)) {

                    MotorSpeed = Range.clip(armspeed, -1, 1);

                    //set speed to preset armspeed
                    arm.setPower(MotorSpeed);

                    currentPositionArm = arm.getCurrentPosition();
                } else {
                    runningOpMode.telemetry.addData("stopping", "");
                    runningOpMode.telemetry.update();
                    armstates = states.STATIONARY;
                }
                break;

            case STATIONARY:
                armtarget = 0;
                armspeed = 0;

                arm.setPower(0);

            default:
                break;
        }
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
        double currentPositionLeft = LEFT_ENCODER_FACTOR * leftFront.getCurrentPosition();
        double currentPositionRight = RIGHT_ENCODER_FACTOR * rightFront.getCurrentPosition();
//
//        logger.write("currentPositionLeft " + currentPositionLeft);
//        logger.write("currentPositionRight " + currentPositionRight);

        //set the new targets by adding them to the current position
        double newTargetLeft = target + currentPositionLeft;
        double newTargetRight = target + currentPositionRight;

        boolean TargetNegative = newTargetRight < currentPositionRight;

//        logger.write("newTargetLeft: " + newTargetLeft);
//        logger.write("newTargetRight: " + newTargetRight);

        ElapsedTime Timer = new ElapsedTime();
        Timer.reset();

        double lastEncoderReading = currentPositionRight;
        int numReads = 1;

        //loop will run while the encoder values are within 15 counts of their target & the opmode is active
        while (runningOpMode.opModeIsActive() && Math.abs(newTargetLeft - currentPositionLeft) > 200
                && Math.abs(newTargetRight - currentPositionRight) > 200 && evaldistance(currentPositionRight, newTargetRight, TargetNegative, 200)
                && Timer.milliseconds() < Timeout) {

//            if(newTargetLeft - leftFront.getCurrentPosition() > 15)
//                break;

            //get the heading
            double drift = getHeading();

            //set the speed of the motors according to the drift from the gyro target
            double leftSpeed = (speedLeft + (drift - gyroTarget) / 100);
            double rightSpeed = (speedRight - (drift - gyroTarget) / 100);


            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            leftRear.setPower(leftSpeed);
            rightFront.setPower(rightSpeed); //no encoders
            leftFront.setPower(leftSpeed); //no encoders
            rightRear.setPower(rightSpeed);

            currentPositionLeft = LEFT_ENCODER_FACTOR * leftFront.getCurrentPosition();
            currentPositionRight = RIGHT_ENCODER_FACTOR * rightFront.getCurrentPosition();

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

        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);

//        logger.write("Average delta: " + (lastEncoderReading / numReads));
//        logger.write("Distance between target and actual position: " + (newTargetRight - currentPositionRight));

        runningOpMode.telemetry.addData("Status", "Complete");
        runningOpMode.telemetry.update();

//        logger.printDivider();

    }

    public void strafe(double leftSpeed, double rightSpeed, double target, double gyroTarget, StrafeDirection direction) throws InterruptedException {

        /* Function for driving forward with encoders */

        double adjustedLeftSpeed = leftSpeed;
        double adjustedRightSpeed = rightSpeed;

//        Stopwatch timer = Stopwatch.createStarted();
        double currentPosition = leftRear.getCurrentPosition();
//
//        logger.write("currentPositionLeft " + currentPositionLeft);
//        logger.write("currentPositionRight " + currentPositionRight);

        //set the new targets by adding them to the current position
        if(direction == StrafeDirection.Left) {
            target *= -1;
        }

        double newTarget = target + currentPosition;

        boolean TargetNegative = newTarget < currentPosition;

//        logger.write("newTargetLeft: " + newTargetLeft);
//        logger.write("newTargetRight: " + newTargetRight);

        ElapsedTime Timer = new ElapsedTime();
        Timer.reset();

       // logger.write("Start Position: " + currentPosition);
        //loop will run while the encoder values are within 15 counts of their target & the opmode is active
        while (runningOpMode.opModeIsActive() && Math.abs(newTarget - currentPosition) > 200
                && evaldistance(currentPosition, newTarget, TargetNegative, 200)) {

//            if(useDistance && distance2.getDistance(DistanceUnit.CM) <= 2.7 && distance1.getDistance(DistanceUnit.CM) <= 2.7)
//                distanceReached = true;

            //get the heading
            //double drift = getHeading();

            switch(direction) {
                case Right:
                    //set the speed of the motors according to the drift from the gyro target
                    //adjustedLeftSpeed = (-leftSpeed + (drift - gyroTarget) / 100);
                    //adjustedRightSpeed = (-rightSpeed - (drift - gyroTarget) / 100);


                    //adjustedLeftSpeed = Range.clip(adjustedLeftSpeed, -1, 1);
                    //adjustedRightSpeed = Range.clip(adjustedRightSpeed, -1, 1);

//                    leftRear.setPower(adjustedLeftSpeed);
//                    rightFront.setPower(adjustedRightSpeed); //no encoders
                    leftRear.setPower(-leftSpeed);
                    rightFront.setPower(-rightSpeed); //no encoders
                    leftFront.setPower(leftSpeed); //no encoders
                    rightRear.setPower(rightSpeed);
                    break;
                case Left:
//                    adjustedLeftSpeed = (leftSpeed + (drift - gyroTarget) / 100);
//                    adjustedRightSpeed = (rightSpeed - (drift - gyroTarget) / 100);
//
//
//                    adjustedLeftSpeed = Range.clip(adjustedLeftSpeed, -1, 1);
//                    adjustedRightSpeed = Range.clip(adjustedRightSpeed, -1, 1);

//                    leftRear.setPower(adjustedLeftSpeed);
//                    rightFront.setPower(adjustedRightSpeed); //no encoders
                    leftRear.setPower(leftSpeed);
                    rightFront.setPower(rightSpeed); //no encoders
                    leftFront.setPower(-leftSpeed); //no encoders
                    rightRear.setPower(-rightSpeed);
                    break;
            }
            currentPosition = leftRear.getCurrentPosition();
            runningOpMode.telemetry.addData("Status", "Busy");
            //runningOpMode.telemetry.addData("heading", drift);
            runningOpMode.telemetry.addData("position", currentPosition);
            //logger.write("Position: " + currentPosition);
            runningOpMode.telemetry.update();
        }
       // logger.write("End position: " + currentPosition);
        //logger.write("End");

        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);

        //telemetry.addData("Status", "Complete");
        //telemetry.update();

    }




    public void armReset() {

        while (runningOpMode.opModeIsActive() && armstop.getState() == true) {
            arm.setPower(0.8);
        }
        if(armstop.getState() == false){
            arm.setPower(0);

        }

    }

    public void slidesDown(long timeout) {
        long startTime = System.currentTimeMillis();

        leftslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftslide.setPower(-0.9);
        rightslide.setPower(-0.9);

        while (magswitch.getState() == true && System.currentTimeMillis() - startTime < timeout) {
            // continue loop until magswitch is false or timeout has been reached
        }

        leftslide.setPower(0);
        rightslide.setPower(0);
        leftslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public void zeroArm() {
//
//        while (runningOpMode.opModeIsActive() && arm.getCurrentPosition() > 5) {
//            arm.setPower(-0.8);
//        }
//
//    }

    public void armreset() {
        if (armstop.getState() == true) {
            arm.setPower(-0.85); // move the arm down
        } else if (armstop.getState() == false) {
            arm.setPower(0);
        } else {
            arm.setPower(0); // stop the arm
        }
    }

    public void armrotate(double motorSpeed, int target) {

        /* Function for driving forward with encoders */

        double MotorSpeed = motorSpeed;

        //get initial positions for encoders
        double currentPositionarm = arm.getCurrentPosition();


        //set the new targets by adding them to the current position
        double newTargetarmrotate = target + currentPositionarm;

//        logger.write("newTargetarmrotate " + target);
         logger.write("Arm start: " + currentPositionarm);

        //loop will run while the encoder values are within 15 counts of their target & the opmode is active


        while (runningOpMode.opModeIsActive() && Math.abs(newTargetarmrotate - arm.getCurrentPosition()) > 300
             &&  arm.getCurrentPosition() - currentPositionarm < 300){


            //clip the range of the motor powers to make sure it doesn't exceed 1
            MotorSpeed = Range.clip(MotorSpeed, -1, 1);


            arm.setPower(motorSpeed);
            runningOpMode.telemetry.addData("Status", "Busy");
            runningOpMode.telemetry.addData("currentPosRight", currentPositionarm);
            //log.write("armrotate: " + turret.getCurrentPosition());
            runningOpMode.telemetry.update();

            currentPositionarm = arm.getCurrentPosition();
            //logger.write("Arm pos: " + currentPositionarm);
        }
        logger.write("Arm end: " + currentPositionarm);

        runningOpMode.idle();
        arm.setPower(0);

    }




    public void strafeWithSeconds(boolean strafeLeft, double leftspeed, double rightspeed, int sleep) {

        if (strafeLeft) {

            leftFront.setPower(-leftspeed);
            leftRear.setPower(leftspeed);
            rightFront.setPower(rightspeed);
            rightRear.setPower(-rightspeed);
        } else {
            leftFront.setPower(leftspeed);
            leftRear.setPower(-leftspeed);
            rightFront.setPower(-rightspeed);
            rightRear.setPower(rightspeed);
        }

        runningOpMode.sleep(sleep);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        runningOpMode.telemetry.addData("Exiting", "Strafe with Seconds");
        runningOpMode.telemetry.update();

    }

    public void setdrivetrainbehavior(DcMotor.ZeroPowerBehavior floatbehavior) {
        leftFront.setZeroPowerBehavior(floatbehavior);
        rightFront.setZeroPowerBehavior(floatbehavior);
        leftRear.setZeroPowerBehavior(floatbehavior);
        rightRear.setZeroPowerBehavior(floatbehavior);

    }


    public void driveStraightWithSeconds(double leftSpeed, double rightSpeed, int sleep) {
        leftFront.setPower(leftSpeed);
        leftRear.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        rightRear.setPower(rightSpeed);
        runningOpMode.sleep(sleep);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }


    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = angles.firstAngle;

        if (heading < 0) {
            heading += 360;
        }

        return heading;

    }

    public void turn(double leftSpeed, double rightSpeed, int degrees, int error,
                     boolean is360) throws InterruptedException {

//        logger.write("turn");

        //right requires negative power

        //get beginning angle
        double currentAngle = getHeading();

        runningOpMode.telemetry.addData("currentAngle", currentAngle);
        runningOpMode.telemetry.update();

//        logger.write("currentAngle " + currentAngle);

        //while the difference between the target angle and the current angle is greater than the error & opMode is still running
        while (Math.abs(currentAngle - degrees) > error && runningOpMode.opModeIsActive()) {

            //if we want to convert to 360, we'll use getAngle() method
            if (!is360) {
                currentAngle = getHeading();
            } else {
                currentAngle = getAngle();
            }

            //if the current angle is greater than the target angle, turn right
            if (currentAngle > degrees) {
                leftRear.setPower(leftSpeed);
                rightFront.setPower(-rightSpeed);
                leftFront.setPower(leftSpeed);
                rightRear.setPower(-rightSpeed);
            }
            //if it's less than it, turn left
            if (currentAngle < degrees) {
                leftRear.setPower(-leftSpeed);
                rightFront.setPower(rightSpeed);
                leftFront.setPower(-leftSpeed);
                rightRear.setPower(rightSpeed);
            }

            runningOpMode.telemetry.addData("currentAngle", currentAngle);
            runningOpMode.telemetry.update();

//            logger.write("currentAngle " + currentAngle);
        }

        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);

//        logger.printDivider();
    }

    public double getHeading() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = angles.firstAngle;

        return heading;
    }

    public static boolean evaldistance(double currentpos, double target, boolean TargetSmall, int threshold) {

        //logger.write("EvalDistance");

        //if the target is smaller than the current position, this should be true

        if (TargetSmall) {
//            logger.write(currentpos - target);
            return currentpos - target > threshold;
        } else {
            return currentpos - target < threshold;

        }
    }


    public boolean Sensor(ColorSensor colorSensor) {

        if ((colorSensor.blue() < 3000)) {
            return true;
        } else {
            return false;
        }


    }

    public boolean SensorRed(ColorSensor colorSensor) {

        if ((colorSensor.red() < 6000)) {
            return true;
        } else {
            return false;
        }

    }

//    public boolean SensorGreen(ColorSensor colorSensor) {
//
//        if ((colorSensor.green() < 4000)) {
//            return true;
//        } else {
//            return false;
//        }
//
//    }

    public static ParkZone GetParkZone() {

        int Red = colordetection.red();
        int Blue = colordetection.blue();
        int Green = colordetection.green();

        if (Red > Blue && Red > Green) {
            return ParkZone.LeftZone;
        } else if (Blue > Red && Blue > Green) {
            return ParkZone.CenterZone;
        } else if (Green > Blue && Green > Red) {
            return ParkZone.RightZone;
        }

        return ParkZone.CenterZone;

    }


//    public void driveWithDistance (double leftspeed, double rightspeed) {
//        while (runningOpMode.opModeIsActive()&& distanceright.getDistance(DistanceUnit.CM) > 1.0 && distanceright.getDistance(DistanceUnit.CM) < 11.0) {
//
//
//
//            leftRear.setPower(leftspeed);
//            rightRear.setPower(rightspeed);
//            rightFront.setPower(rightspeed);
//            leftFront.setPower(leftspeed);
//
////            log.write(distanceright.getDistance(DistanceUnit.CM));
//
//        }
//        leftFront.setPower(0);
//        leftRear.setPower(0);
//        rightFront.setPower(0);
//        rightRear.setPower(0);
//    }

//    public void correctiondrive (double power, double facing) {
//        //direction is 1 for forwards and -1 for backwards
//        //direction is set to forwards by default
//        do {
//            leftFront.setPower(power * facing);
//            rightFront.setPower(power * facing);
//            leftRear.setPower(power * facing);
//            rightRear.setPower(power * facing);
//        } while (runningOpMode.opModeIsActive()&& distanceright.getDistance(DistanceUnit.CM) > 11);
//    }
//
//    public void correctionstrafe (double maxdistance, double mindistance) {
//        //if the distance right is greater than maxdistance then it strafes right until it's less than or equal to
//        //if the distance right is less than mindistance it strafes left until it's greater than or equal to
//        while (distanceright.getDistance(DistanceUnit.CM) > maxdistance) {
//            strafeWithSeconds(false,0.1,0.1,800);
//        } while (distanceright.getDistance(DistanceUnit.CM) < mindistance) {
//            strafeWithSeconds(true,0.1,0.1,800);
//        }
//    }
//
//    public void strafetoPole (double leftspeed, double rightspeed) {
//        while (runningOpMode.opModeIsActive() && distanceright.getDistance(DistanceUnit.CM) < 7.0 ) {
//            leftRear.setPower(-leftspeed);
//            rightRear.setPower(rightspeed);
//            rightFront.setPower(-rightspeed);
//            leftFront.setPower(leftspeed);
//
////          log.write(distanceright.getDistance(DistanceUnit.CM));
//
//        }
//        leftFront.setPower(0);
//        leftRear.setPower(0);
//        rightFront.setPower(0);
//        rightRear.setPower(0);
//    }



    public int linearslides(double motorSpeed, int target) throws InterruptedException {

        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double MotorSpeed = motorSpeed;

        //get initial positions for encoders
        double currentPositionlift = leftslide.getCurrentPosition();


        //set the new targets by adding them to the current position
        double newTargetlift = target + currentPositionlift;


        //loop will run while the encoder values are within 15 counts of their target & the opmode is active
        while (runningOpMode.opModeIsActive() && Math.abs(newTargetlift - leftslide.getCurrentPosition()) > 300
                && leftslide.getCurrentPosition() - currentPositionlift < 300) {



            //get the heading (current gyro angle)

            //set the speed of the motors according to the drift from the gyro target


            //clip the range of the motor powers to make sure it doesn't exceed 1
            MotorSpeed = Range.clip(MotorSpeed, -1, 1);


            leftslide.setPower(motorSpeed);
            rightslide.setPower(motorSpeed);
            runningOpMode.telemetry.addData("Status", "Busy");
            runningOpMode.telemetry.addData("currentPosRight", leftslide.getCurrentPosition());
            runningOpMode.telemetry.update();
        }
        leftslide.setPower(0);
        rightslide.setPower(0);


        runningOpMode.telemetry.addData("Status", "Complete");
        runningOpMode.telemetry.update();

        return target;
    }

//    public int armmovement(double motorSpeed, int target) throws InterruptedException {
//
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        double MotorSpeed = motorSpeed;
//
//        //get initial positions for encoders
//        double currentPositionlift = arm.getCurrentPosition();
//
//
//        //set the new targets by adding them to the current position
//        double newTargetlift = target + currentPositionlift;
//
//
//        //loop will run while the encoder values are within 15 counts of their target & the opmode is active
//        while (runningOpMode.opModeIsActive() && Math.abs(newTargetlift - arm.getCurrentPosition()) > 300
//                && arm.getCurrentPosition() - currentPositionlift < 300) {
//
//
//
//            //get the heading (current gyro angle)
//
//            //set the speed of the motors according to the drift from the gyro target
//
//
//            //clip the range of the motor powers to make sure it doesn't exceed 1
//            MotorSpeed = Range.clip(MotorSpeed, -1, 1);
//
//
//            arm.setPower(motorSpeed);
////            rightslide.setPower(motorSpeed);
//            runningOpMode.telemetry.addData("Status", "Busy");
//            runningOpMode.telemetry.addData("currentPosRight", leftslide.getCurrentPosition());
//            runningOpMode.telemetry.update();
//        }
//        arm.setPower(0);
////        rightslide.setPower(0);
//
//
//        runningOpMode.telemetry.addData("Status", "Complete");
//        runningOpMode.telemetry.update();
//
//        return target;
//    }



}
