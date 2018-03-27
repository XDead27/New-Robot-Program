package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Purplecoder on 27/01/2018.
 */

@Autonomous(name = "Red_Autonom", group = "Autonomous")
//@Disabled
public class Red_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Done!", "play");
        telemetry.update();
    }

    @Override
    protected void runOp() throws InterruptedException {
        ball_auto(true);

        grab_cube(true);

        wait(1.0);

        cubes_to_position(1, 1500);

        wait(1.0);

        //rotate_ticks(800, -1);
        move(0.4, -0.4); //stanga
        wait(0.05);

        wait(1.0);

        //run_forward(7000, 1);

        move(-0.6, -0.6);
        wait(2.0);

        //rotate_ticks(400, 1);

        move(0.3, -0.3); //stanga
        wait(0.05);

        cubes_to_position(-1, 0);

        wait(0.5);

        grab_cube(false);

        move(-0.5, -0.5);
        wait(0.5);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    @Override
    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}