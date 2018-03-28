package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Purplecoder 27/01/2018.
 */

/*****************************
 * MAIN BRANCH
 *****************************/

public abstract class AutonomousMode extends LinearOpMode {
    // Motors
    protected DcMotor cubesMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;

    // Servos
    protected Servo servoArm = null;
    protected Servo servoColor = null;
    protected Servo servoCubesLeft = null;
    protected Servo servoCubesRight = null;

    // Sensors
    protected ModernRoboticsI2cGyro gyroSensor = null;
    protected ColorSensor colorSensor = null;
    //protected OpticalDistanceSensor odsSensor = null;
    //protected ModernRoboticsI2cRangeSensor rangeSensor = null;

    // Constants
    protected static final double ARM_UP = 0.96;
    protected static final double ARM_DOWN = 0.33;
    protected static final double COLOR_FORWARD = 0.0;
    protected static final double COLOR_BACK = 1.0;
    protected static final double MID_SERVO = 0.5;
    protected static final double CUBES_MIN = 0.65;
    protected static final double CUBES_MAX = 0.8;
    protected static final double LIFT_MAX = 5000;
    protected static final double COUNTS_PER_CM = 67;

    protected ElapsedTime runtime = new ElapsedTime();

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map the motors
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        // Map the servos
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("color");
        servoCubesLeft = hardwareMap.servo.get("cubes_left");
        servoCubesRight = hardwareMap.servo.get("cubes_right");
        // Map the sensors
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cubesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cubesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cubesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesRight.setPosition(CUBES_MAX);
        servoCubesLeft.setPosition(CUBES_MAX);

        // Calibrate sensors
        colorSensor.enableLed(true);
        gyroSensor.calibrate();
        while(!isStopRequested() && gyroSensor.isCalibrating() && opModeIsActive()){
            idle();
        }

        telemetry.addData("Status", "Gyro calibrated");
        telemetry.update();

        waitForStart();

        runOp();

        endOp();
    }

    protected abstract void runOp();
    protected abstract void endOp();

    //**********
    //Methods
    //**********
    protected void Power_Wheels(double LMotorsPower, double RMotorsPower){
        leftMotorF.setPower(LMotorsPower);
        leftMotorB.setPower(LMotorsPower);
        rightMotorF.setPower(RMotorsPower);
        rightMotorB.setPower(RMotorsPower);
    }

    protected void Move_Wheels_With_Encoders(double power, int target){

        Range.clip(power, 0, 0.9);

        leftMotorF.setTargetPosition(target);
        leftMotorB.setTargetPosition(target);
        rightMotorF.setTargetPosition(target);
        rightMotorB.setTargetPosition(target);
        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Power_Wheels(power, power);

        while(leftMotorF.isBusy() || leftMotorB.isBusy() || rightMotorF.isBusy() || rightMotorB.isBusy()){
            idle();
        }

        Stop_Wheels();
    }

    protected void Rotate_Wheels_Gyro(double power, double direction, double gyrotarget){
        Range.clip(gyrotarget, -180, 180);
        Range.clip(direction, -1,1);
        Range.clip(power, 0, 0.9);
        int initHeading = gyroSensor.getHeading();

        Power_Wheels(direction*power, direction*(-power) );

    }

    protected void Stop_Wheels(){
        Power_Wheels(0, 0);
    }

}