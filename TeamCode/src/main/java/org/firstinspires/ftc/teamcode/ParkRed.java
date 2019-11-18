package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ParkRed")
public class ParkRed extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        waitForStart();
       strafeRight(.7,800,500);
        while(true){
            if(robot.sensorColor.red() <= 40){
                moveForward(.3,75,75);

            }else if(robot.sensorColor.red() > 40){
                break;
            }
        }
        stopRoobot();

    }
    public void moveForward(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void moveBackWard(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeLeft (double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeRight(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void stopRoobot(){
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }
}
