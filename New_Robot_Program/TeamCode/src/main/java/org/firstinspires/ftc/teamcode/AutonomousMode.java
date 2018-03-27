package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
/**
 * Created by Purplecoder 27/01/2018.
 */

public abstract class AutonomousMode extends LinearOpMode {
    // Motors
    protected DcMotor relicMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;

    // Servos
    protected Servo servoArm = null;
    protected Servo servoColor = null;
    protected Servo servoCubesLeftUp = null;
    protected Servo servoCubesLeftDown = null;
    protected Servo servoCubesRightUp = null;
    protected Servo servoCubesRightDown = null;
    protected Servo servoRelic = null;
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

//actioneaza servoCubesRightDown si servoCubesLeftDown,actioneaza servoArm,citeste culoarea,actioneaza servoColor,foloseste OpenGLMatrix
//afla pozitia cubului,actioneaza motoarele,actioneaza leftMotorF/B sau rightMotorF/B(pentru a se alinia cu cryptobox)
//actioneaza toate motoarele mai putin cubesMotor,folosinf gyroSensor face o rotatie de 180^,actioneaza toate motoarele mai putin relicMotor
//actioneaza servoCubes..,actioneaza toate motoarele mai putin relicMotor cu -

 public void runOpMode() {

     rightMotorB.setDirection(DcMotor.Direction.FORWARD);
     rightMotorF.setDirection(DcMotor.Direction.FORWARD);
     leftMotorF.setDirection(DcMotor.Direction.REVERSE);
     leftMotorB.setDirection(DcMotor.Direction.REVERSE);

     while(opModeIsActive()){
        waitForStart();

            MoveForward(5);
            TurnLeft(3);
            TurnRight(3);
            }

     }
    public void MoveForward(double power){
        leftMotorF.setPower(power);
        leftMotorB.setPower(power);
        rightMotorB.setPower(power);
        rightMotorF.setPower(power);
    }

    public void TurnRight(double power){
        leftMotorF.setPower(power);
        leftMotorB.setPower(power);
        rightMotorB.setPower(-power);
        rightMotorF.setPower(-power);
        gyroSensor.getHeading();
        gyroSensor.rawX();
    }
    public void TurnLeft(double power){
        leftMotorF.setPower(-power);
        leftMotorB.setPower(-power);
        rightMotorB.setPower(power);
        rightMotorF.setPower(power);
        gyroSensor.getHeading();
        gyroSensor.rawX();
    }


}
































 }
 }