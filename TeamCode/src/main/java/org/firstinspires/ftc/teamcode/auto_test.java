package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "autotest")
public class auto_test extends LinearOpMode {
        components robot = new components();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);


        waitForStart();

           // telemetry.addData("Red  ", robot.sensorColor.red());
            robot.backwards(2,.2);
            // go to foundation
            sleep(1000);
            robot.strafeleft(30.2,.1);
            //latch on
             robot.drag.setPosition(.1);
            //go back to building zone
            // robot.straferight(27.2,.2);
            //unlatch
            // robot.drag.setPosition(0);

    }
}
