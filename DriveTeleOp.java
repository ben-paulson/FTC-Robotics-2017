/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive", group="Iterative Opmode")

public class DriveTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime driveTime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive_front = null;
    private DcMotor rightDrive_front = null;

    private DcMotor elevator = null;
    private Servo clawServo = null;
    private Servo clawServoRight = null;
    //private Servo clawServoTop = null;
    private Servo clawServoRightTop = null;
    //private Servo pushServo = null;
    private Servo clawServoStack = null;

    private ColorSensor colorSensor = null;
    private Servo colorServoUpDown = null;
    private Servo colorServoSideSide = null;

    private navXPIDController yawPIDController;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;


    // Runs once after hitting init
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive_front  = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightDrive_front = hardwareMap.get(DcMotor.class, "right_drive_front");


        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServoRight = hardwareMap.get(Servo.class, "claw_servo_right");
        //clawServoTop = hardwareMap.get(Servo.class, "claw_servo_top");
        clawServoRightTop = hardwareMap.get(Servo.class, "claw_servo_right_top");
        clawServoStack = hardwareMap.get(Servo.class, "claw_servo_stack");
        //clawServoTop.setDirection(Servo.Direction.REVERSE);
        clawServoRightTop.setDirection(Servo.Direction.REVERSE);
        //pushServo = hardwareMap.get(Servo.class, "push_servo");

        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorServoUpDown = hardwareMap.get(Servo.class, "color_servo_up_down");
        colorServoSideSide = hardwareMap.get(Servo.class, "color_servo_side_side");


        // Left side motors need to be set in reverse to go same way as right motors);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive_front.setDirection(DcMotor.Direction.REVERSE);
        rightDrive_front.setDirection(DcMotor.Direction.FORWARD);

        clawServo.scaleRange(0.15, 1.0);
        clawServoRight.scaleRange(0.0, 0.8);
        //clawServoTop.scaleRange(0.15, 1.0);
        //clawServoRightTop.scaleRange(0.0, 0.8);
        clawServoStack.scaleRange(0.0, 0.15);

        telemetry.addData("LinearOp", "About to get AHRS instance.");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        telemetry.addData("LinearOp", "Got AHRS Instance.");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    // Runs repeatedly after hitting init but before hitting play
    @Override
    public void init_loop() {
        clawServo.setPosition(0);
        colorServoSideSide.setPosition(0.4);
        colorServoUpDown.setPosition(0);
    }

    // Runs once after hitting play
    @Override
    public void start() {driveTime.reset();}

    // Runs repeatedly after hitting play but before hitting stop
    @Override
    public void loop() {

        // Motor power
        double leftPower;
        double rightPower;

        // Servo
        double elevatorSpeed = -gamepad2.left_stick_y / 2.0;
        boolean clawDown = gamepad2.a;
        boolean pushServoDown = gamepad2.b;

        // Left gamepad stick controls fwd/bkwd, right stick controls turn
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        /*if (!gamepad1.atRest()) {
            gamepad1.refreshTimestamp();

            if (driveTime.milliseconds() - gamepad1.timestamp < 1000) {
                drive *= (driveTime.milliseconds() - gamepad1.timestamp) / 1000.0;
                turn *= (driveTime.milliseconds() - gamepad1.timestamp) / 1000.0;
            }
        }*/


        if (turn > 0.5) {
            turn = 0.5;
        }
        if (turn < -0.5) {
            turn = -0.5;
        }

        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        telemetry.addData("Power", "Right: " + rightPower);
        telemetry.addData("Power", "Left: " + leftPower);

        // power to servos


        if (clawDown) {
            telemetry.addData("claw", "down");
            clawServo.setPosition(0);
            clawServoRight.setPosition(1);
            //clawServoTop.setPosition(0);
            clawServoRightTop.setPosition(1);
            clawServoStack.setPosition(0);
        } else {
            clawServo.setPosition(1);
            clawServoRight.setPosition(0);
            //clawServoTop.setPosition(1);
            clawServoRightTop.setPosition(0);
            clawServoStack.setPosition(1.0);
        }

        /*if (pushServoDown) {
            pushServo.setPosition(1);
        } else {
            pushServo.setPosition(0);
        }*/

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftDrive_front.setPower(leftPower);
        rightDrive_front.setPower(rightPower);

        elevator.setPower(elevatorSpeed);

        // Brake if no power in motor (don't drift)'

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + driveTime.toString());
        // Amount of blue detected by color sensor
        telemetry.addData("Blue detected", "Blue: " + colorSensor.blue());
        telemetry.addData("Red detected", "Red: " + colorSensor.red());

    }

    // Runs once after hitting stop
    @Override
    public void stop() {
    }



}
