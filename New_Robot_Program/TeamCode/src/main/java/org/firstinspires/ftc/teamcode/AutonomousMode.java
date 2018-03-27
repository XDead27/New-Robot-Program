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



}