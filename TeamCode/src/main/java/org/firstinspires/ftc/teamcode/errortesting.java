package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class errortesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        Jake_2_Hardware robot = new Jake_2_Hardware();

        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){

            robot.MotorVL.setPower(gamepad1.left_stick_y);
            robot.MotorVR.setPower(gamepad1.left_stick_y);

        }

    }
}
