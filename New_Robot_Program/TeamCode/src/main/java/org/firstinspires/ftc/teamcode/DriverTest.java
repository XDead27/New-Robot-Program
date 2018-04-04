package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    //protected Servo servoClaw = null;
    //protected Servo servoExtension  = null;
    protected Servo servoArm = null;
    protected Servo servoColor = null;
    protected Servo servoCubesDownLeft = null;
    protected Servo servoCubesDownRight = null;
    protected Servo servoCubesUpRight = null;
    protected Servo servoCubesUpLeft = null;

    //Constants
    protected static final double CLAW_UP = 0.0;
    protected static final double CLAW_DOWN = 0.5;
    protected static final double EXTENSION_UP = 0.0;
    protected static final double EXTENSION_MID = 0.5;
    protected static final double EXTENSION_DOWN = 1.0;
    protected static final double ARM_UP = 0.96;
    protected static final double ARM_DOWN = 0.33;
    protected static final double COLOR_FORWARD = 0.0;
    protected static final double COLOR_BACK = 1.0;
    protected static final double MID_SERVO = 0.5;
    protected static final double CUBES_MIN = 0.65;
    protected static final double CUBES_MAX = 0.8;
    protected static final double LIFT_MAX = 5000;
    protected static final double COUNTS_PER_CM = 67;
    protected static final double MAX_P_SPEED = 1/360;
    // Additional helper variables
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;
    double currLeftWheelsPower = 0;
    double currRightWheelsPower = 0;
    double WheelsIncrementing = 0.1;

    private double cubesPower = 1;

    //private boolean switchServoCubesLeft = false;
    //private boolean switchServoCubesRight = false;
    private boolean switchServoCubes = false;

    //@Override
    public void runOpMode() throws InterruptedException {
        // Map the motors
        relicLMotor = hardwareMap.dcMotor.get("relic_left");
        relicRMotor = hardwareMap.dcMotor.get("relic_right");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        // Map the servos
        //servoClaw = hardwareMap.servo.get("claw");
        //servoExtension = hardwareMap.servo.get("extension");
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("colorS");
        servoCubesDownLeft = hardwareMap.servo.get("cubes_down_left");
        servoCubesDownRight = hardwareMap.servo.get("cubes_down_right");
        servoCubesUpLeft = hardwareMap.servo.get("cubes_up_left");
        servoCubesUpRight = hardwareMap.servo.get("cubes_up_right");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.FORWARD);
        leftMotorB.setDirection(DcMotor.Direction.FORWARD);
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);

        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cubesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        relicLMotor.setDirection(DcMotor.Direction.FORWARD);
        relicRMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set servo directions
        //servoClaw.setDirection(Servo.Direction.REVERSE);
        //servoExtension.setDirection(Servo.Direction.FORWARD);
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesDownLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesDownRight.setDirection(Servo.Direction.REVERSE);
        servoCubesUpLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesUpRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        relicLMotor.setPower(0);
        relicRMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
        // Initialize servo positions
        //servoClaw.setPosition(CLAW_DOWN);
        //servoExtension.setPosition(EXTENSION_UP);
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_BACK);
        servoCubesDownLeft.setPosition(CUBES_MAX);
        servoCubesDownRight.setPosition(CUBES_MAX);
        servoCubesUpLeft.setPosition(CUBES_MAX);
        servoCubesUpRight.setPosition(CUBES_MAX);

        telemetry.addData("Say", "Hello Driver!");
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run the motors.");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {

            /*if(gamepad1.left_stick_x != 0 && gamepad1.right_bumper){
                if(cubesMotor.getCurrentPosition() < LIFT_MAX && cubesMotor.getCurrentPosition() > 0) {
                    cubesMotor.setPower(gamepad1.left_stick_x);
                }else{
                    cubesMotor.setPower(0);
                }
            }*/

            if(Math.abs(gamepad1.left_stick_x) > deadzone){
                if(currLeftWheelsPower < gamepad1.left_stick_x){
                    currLeftWheelsPower += WheelsIncrementing;
                }
                if(currLeftWheelsPower > gamepad1.left_stick_x){
                    currLeftWheelsPower -= WheelsIncrementing;
                }

                Power_Left_Wheels(currLeftWheelsPower);
            }else if(Math.abs(currLeftWheelsPower) >= 0){
                if(currLeftWheelsPower < 0){
                    currLeftWheelsPower += WheelsIncrementing;
                }
                if(currLeftWheelsPower > 0){
                    currLeftWheelsPower -= WheelsIncrementing;
                }
                Power_Left_Wheels(currLeftWheelsPower);
            }


            if(Math.abs(gamepad1.right_stick_x) > deadzone){
                if(currRightWheelsPower < gamepad1.right_stick_x){
                    currRightWheelsPower += WheelsIncrementing;
                }
                if(currRightWheelsPower > gamepad1.right_stick_x){
                    currRightWheelsPower -= WheelsIncrementing;
                }

                Power_Right_Wheels(currRightWheelsPower);
            }else if(Math.abs(currRightWheelsPower) >= 0){
                if(currRightWheelsPower < 0){
                    currRightWheelsPower += WheelsIncrementing;
                }
                if(currRightWheelsPower > 0){
                    currRightWheelsPower -= WheelsIncrementing;
                }
                Power_Right_Wheels(currRightWheelsPower);
            }

            if(gamepad1.a){
                servoCubesUpLeft.setPosition(CUBES_MAX);
                servoCubesUpRight.setPosition(CUBES_MAX);
            }
            if(gamepad1.b){
                servoCubesUpLeft.setPosition(CUBES_MIN);
                servoCubesUpRight.setPosition(CUBES_MIN);
            }

        }
    }

    private void Power_Left_Wheels(double LMotorsPower){
        Range.clip(LMotorsPower,0,0.9);
        leftMotorB.setPower(LMotorsPower);
        leftMotorF.setPower(LMotorsPower);
    }

    private void Power_Right_Wheels(double RMotorsPower){
        Range.clip(RMotorsPower,0,0.9);
        rightMotorB.setPower(RMotorsPower);
        rightMotorF.setPower(RMotorsPower);
    }

}

