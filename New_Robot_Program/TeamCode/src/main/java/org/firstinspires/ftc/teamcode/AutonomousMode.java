package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*****************************
 * MAIN BRANCH
 *****************************/

public abstract class AutonomousMode extends LinearOpMode {
    // Motors
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

    // Sensors
    protected ModernRoboticsI2cGyro gyroSensor = null;
    protected ColorSensor colorSensor = null;

    // Constants
    protected static final double CLAW_UP = 0.0;
    protected static final double CLAW_DOWN = 0.5;
    protected static final double EXTENSION_UP = 0.0;
    protected static final double EXTENSION_MID = 0.5;
    protected static final double EXTENSION_DOWN = 1.0;
    protected static final double ARM_UP = 0.96;
    protected static final double ARM_DOWN = 0.40;
    protected static final double COLOR_FORWARD = 0.0;
    protected static final double COLOR_BACK = 1.0;
    protected static final double MID_SERVO = 0.5;
    protected static final double CUBES_MIN = 0.65;
    protected static final double CUBES_MAX = 0.8;
    protected static final double COUNTS_PER_CM = 67;
    protected static final double LIFT_MAX = 36 * COUNTS_PER_CM;
    protected static final double MAX_P_SPEED = 1/360;

    protected ElapsedTime runtime = new ElapsedTime();

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
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
        servoClaw = hardwareMap.servo.get("claw");
        servoExtension = hardwareMap.servo.get("extension");
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("colorS");
        servoCubesDownLeft = hardwareMap.servo.get("cubes_down_left");
        servoCubesDownRight = hardwareMap.servo.get("cubes_down_right");
        servoCubesUpLeft = hardwareMap.servo.get("cubes_up_left");
        servoCubesUpRight = hardwareMap.servo.get("cubes_up_right");
        // Map the sensors
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.FORWARD);
        leftMotorB.setDirection(DcMotor.Direction.FORWARD);
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);
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
        relicLMotor.setDirection(DcMotor.Direction.FORWARD);
        relicRMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set servo directions
        servoClaw.setDirection(Servo.Direction.REVERSE);
        servoExtension.setDirection(Servo.Direction.FORWARD);
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
        servoClaw.setPosition(CLAW_DOWN);
        servoExtension.setPosition(EXTENSION_UP);
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesDownLeft.setPosition(CUBES_MAX);
        servoCubesDownRight.setPosition(CUBES_MAX);
        servoCubesUpLeft.setPosition(CUBES_MAX);
        servoCubesUpRight.setPosition(CUBES_MAX);

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

    protected void Release_Cubes(){
        servoCubesDownLeft.setPosition(CUBES_MIN);
        servoCubesDownRight.setPosition(CUBES_MIN);
        servoCubesUpLeft.setPosition(CUBES_MIN);
        servoCubesUpRight.setPosition(CUBES_MIN);
    }

    protected void Read_Push_Ball (boolean RedTeam){

        servoArm.setPosition(ARM_DOWN);
        sleep(1000);

        while(true) {
            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                servoColor.setPosition(RedTeam ? COLOR_BACK : COLOR_FORWARD);
                break;
            } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                servoColor.setPosition(RedTeam ? COLOR_FORWARD : COLOR_BACK);
                break;
            }
        }

        sleep(1000);
        servoColor.setPosition(MID_SERVO);
        servoArm.setPosition(ARM_UP);
        sleep(1000);
        servoColor.setPosition(COLOR_BACK);
    }

    protected void Power_Wheels(double LMotorsPower, double RMotorsPower){
        leftMotorF.setPower(LMotorsPower);
        leftMotorB.setPower(LMotorsPower);
        rightMotorF.setPower(RMotorsPower);
        rightMotorB.setPower(RMotorsPower);
    }

    protected void Move_Wheels_With_Encoders(double power, int target){

        Range.clip(power, 0, 0.9);

        leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotorF.setTargetPosition(target);
        leftMotorB.setTargetPosition(target);
        rightMotorF.setTargetPosition(target);
        rightMotorB.setTargetPosition(target);

        Power_Wheels(power, power);

        while(opModeIsActive() && (leftMotorF.isBusy() && leftMotorB.isBusy() && rightMotorF.isBusy() && rightMotorB.isBusy())){
            telemetry.addData("Busy", leftMotorF.isBusy()? " LF %f" :"", (double)target - leftMotorF.getCurrentPosition());
            telemetry.addData("Busy", leftMotorB.isBusy()? " LB %f" :"", (double)target - leftMotorB.getCurrentPosition());
            telemetry.addData("Busy", rightMotorF.isBusy()? " RF %f" :"", (double)target - rightMotorF.getCurrentPosition());
            telemetry.addData("Busy", rightMotorB.isBusy()? " RB %f" :"", (double)target - rightMotorB.getCurrentPosition());
            telemetry.update();
            idle();
        }

        Stop_Wheels();

        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void Rotate_Wheels_Gyro_PID(double gyrotarget){
        Range.clip(gyrotarget, 0, 355);
        double power;
        double pGain = 0.005;
        double iGain = 0.0005;
        double dGain = 0.04;
        double errorSum = 0;
        double derivative;

        //TODO change gyro sensor to cardinal numbering
        gyroSensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        gyroSensor.resetZAxisIntegrator();

        double error = gyrotarget - (double) gyroSensor.getHeading();
        double d_aux = error;

       do {
            error = gyrotarget - (double)gyroSensor.getHeading();
            if(errorSum + error < 700) {
                errorSum += error;
            }
            derivative = error - d_aux;
            d_aux = error;

            power = error*pGain + errorSum*iGain + derivative*dGain;
            Range.clip(power, 0, 0.8);

            Power_Wheels(-power, power);

            telemetry.addData("Orientation", "%3d", gyroSensor.getHeading());
            telemetry.addData("Error", "%f", error);
            telemetry.addData("ErrorSum", "%f", errorSum);
            telemetry.addData("Derivative", "%f", derivative);
            telemetry.update();

            sleep(5);
        } while(opModeIsActive() && (Math.abs(error) > 0.5));

        Stop_Wheels();

    }

    protected void Stop_Wheels(){
        Power_Wheels(0, 0);
    }

    //Ready for vuforia
}
