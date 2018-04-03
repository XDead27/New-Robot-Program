package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Alin on 12/23/2017.
 */

@TeleOp(name = "DriverTest", group = "TeleOp")
//@Disabled
public class DriverTest extends LinearOpMode {

    protected DcMotor relicLMotor = null;
    protected DcMotor relicRMotor = null;
    protected DcMotor cubesMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;

    // Servos
    protected Servo servoClaw = null;
    protected Servo servoExtension  = null;
    protected Servo servoArm = null;
    protected Servo servoColor = null;
    protected Servo servoCubesDownLeft = null;
    protected Servo servoCubesDownRight = null;
    protected Servo servoCubesUpRight = null;
    protected Servo servoCubesUpLeft = null;

    //Constants
    private static final double ARM_UP = 0.96;
    private static final double ARM_DOWN = 0.12;
    private static final double COLOR_LEFT = 0.0;
    private static final double COLOR_RIGHT = 0.9;
    private static final double MID_SERVO = 0.5;
    private static final double CUBES_MIN = 0.15;
    private static final double CUBES_MAX = 0.35;
    // Additional helper variables
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;

    private double cubesPower = 1;

    //private boolean switchServoCubesLeft = false;
    //private boolean switchServoCubesRight = false;
    private boolean switchServoCubes = false;

    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the motors
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        // Map the servos
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("color");
        servoCubesUpLeft = hardwareMap.servo.get("cubes_up_left");
        servoCubesUpRight = hardwareMap.servo.get("cubes_up_right");
        servoCubesDownLeft = hardwareMap.servo.get("cubes_down_left");
        servoCubesDownRight = hardwareMap.servo.get("cubes_down_right");
        // Set wheel motor directions
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesUpLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesUpRight.setDirection(Servo.Direction.REVERSE);
        servoCubesDownLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesDownRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        cubesMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesUpRight.setPosition(CUBES_MIN);
        servoCubesUpLeft.setPosition(CUBES_MIN);
        servoCubesDownRight.setPosition(CUBES_MIN);
        servoCubesDownLeft.setPosition(CUBES_MIN);

        telemetry.addData("Say", "Hello Driver!");
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run the motors.");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {
            if(Math.abs(gamepad1.left_stick_x) > deadzone){
                Power_Left_Wheels(gamepad1.left_stick_x);
            }else{
                Power_Left_Wheels(0);
            }


            if(Math.abs(gamepad1.right_stick_x) > deadzone){
                Power_Left_Wheels(gamepad1.right_stick_x);
            }else{
                Power_Right_Wheels(0);
            }

        }

    }

    private void Power_Left_Wheels(double LMotorsPower){
        leftMotorB.setPower(LMotorsPower);
        leftMotorF.setPower(LMotorsPower);
    }

    private void Power_Right_Wheels(double RMotorsPower){
        rightMotorB.setPower(RMotorsPower);
        rightMotorF.setPower(RMotorsPower);
    }

}

