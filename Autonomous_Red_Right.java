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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.DecimalFormat;


@Autonomous(name = "Autonomous_Red_Right", group = "Linear Opmode")

public class Autonomous_Red_Right extends LinearOpMode {

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.015; //0.005
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;

    navXPIDController.PIDResult yawPIDResult;

    private ColorSensor color_sensor = null;
    private Servo colorServoUpDown = null;
    private Servo colorServoSideSide = null;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor leftMotor_front = null;
    private DcMotor rightMotor_front = null;


    private DcMotor elevator = null;
    private Servo clawServo = null;
    private Servo clawServoRight = null;


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalRuntime = new ElapsedTime();
    DecimalFormat df;


    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        telemetry.addData("Loop", "Configured PID");


        df = new DecimalFormat("#.##");

        // close the claw
        clawServo.setPosition(0);
        clawServoRight.setPosition(1);
        colorServoSideSide.setPosition(0.4);
        colorServoUpDown.setPosition(0.1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASVGSkD/////AAAAGQEXMZSoJ0iXtg0BNFCj6CUenUCvGOge6mr87iAQ3uRqnVuMswxnbFyiQDhQUSu2yNlTmLAjmHuRZ6sUyp1sqI0dILgDlugfl0orCoUA0MsM5saCes5rNCr6nQbxnemMs9KUhkxQc2KvJXW/SseEN3nRmyitOYsaK4qf6Q+yTqMQ6Ude0hiGeFMb9sVOjJ/MDORG3vmvx4t+LXUkJifh/24LNXaZlpZ7CVKEGNlJC1nBoLYJNhxbMIn0aGqZ3KPJg7hHkRB2mY9jU/dHOJO9RUIy8oo+QFFT8Nm+jvG+T79XiomPFvZ7rpir6tX24LQwmLaNRTRCswgW+ssw3mQ1C3RLn0hfF+fPGGJvtetqDuSC";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        relicTrackables.activate();
        yawPIDResult = new navXPIDController.PIDResult();


        // Calibrate navX
        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }

        telemetry.addData("LinearOp", "navX Calibration Complete");

        navx_device.zeroYaw();
        sleep(200);


        final double TOTAL_RUN_TIME_SECONDS = 2.14;// 1.98

        double drive_speed = 0.4;

        double setElevatorTime = 0.5;


        // Arm
        colorServoUpDown.setPosition(0.65); // half way
        sleep(1500);
        colorServoUpDown.setPosition(0.9);// full way
        sleep(1500);
        colorServoSideSide.setPosition(0.38);
        sleep(1500);
        if (color_sensor.red() * 2 < color_sensor.blue()) {
            colorServoSideSide.setPosition(0.0);
        } else if (color_sensor.blue() * 2 < color_sensor.red()) {
            colorServoSideSide.setPosition(1.0);
        }

        sleep(1000);
        colorServoUpDown.setPosition(0.4);
        sleep(1000);
        colorServoSideSide.setPosition(0.4);
        sleep(1000);
        colorServoUpDown.setPosition(0.1);
        sleep(500);


        // VuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.RIGHT) {

            raiseElevator();
            sleep(500);

            driveToDistance(drive_speed, 24);
            sleep(1000);

            //driveToDistance(drive_speed, 12); // was 0.3
            //sleep(1000);

            turnToDegrees(-30.0, true);
            sleep(500);

            driveToDistance(drive_speed, 4.0); // 2.5
            sleep(200);

            lowerElevator();
            sleep(500);

            clawServo.setPosition(1);
            clawServoRight.setPosition(0);
            sleep(500);

            backUp(drive_speed, 1.5);

            navx_device.close();

        } else if (vuMark == RelicRecoveryVuMark.CENTER) {

            raiseElevator();
            sleep(500);

            driveToDistance(drive_speed, 24);
            sleep(1000);

            turnToDegrees(-90.0, true);
            sleep(1000);

            //backUp(drive_speed, 2.0);
            //sleep(1000);

            //driveToDistance(drive_speed, 18.5); // was 18
            //sleep(1000);

            turnToDegrees(-30.0, false);
            sleep(500);

            driveToDistance(drive_speed, 1);
            sleep(500);

            lowerElevator();
            sleep(500);

            clawServo.setPosition(1);
            clawServoRight.setPosition(0);
            sleep(500);

            backUp(drive_speed, 1.5);

            navx_device.close();

        } else {

            raiseElevator();
            sleep(500);

            driveToDistance(drive_speed, 24); // 18.5
            sleep(1000);

            turnToDegrees(-90.0, true);
            sleep(1000);

            driveToDistance(drive_speed, 5.5);
            sleep(1000);

            turnToDegrees(-25.0, false); // a bit further than 30
            sleep(1000);

            lowerElevator();
            sleep(500);

            clawServo.setPosition(1);
            clawServoRight.setPosition(0);
            sleep(500);

            backUp(drive_speed, 1.5);

            navx_device.close();

        }


    }


    public void initializeHardware() {
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorServoUpDown = hardwareMap.get(Servo.class, "color_servo_up_down");
        colorServoSideSide = hardwareMap.get(Servo.class, "color_servo_side_side");
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor_front = hardwareMap.get(DcMotor.class, "left_drive_front");
        rightMotor_front = hardwareMap.get(DcMotor.class, "right_drive_front");

        elevator = hardwareMap.get(DcMotor.class, "elevator_motor");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServoRight = hardwareMap.get(Servo.class, "claw_servo_right");

        // leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_front.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive_front.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void driveToDistance(double speed, double inches) {

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (inches * 114.65);

        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (leftMotor.isBusy() && rightMotor.isBusy()) {

            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(speed);
                    rightMotor.setPower(speed);
                    leftMotor_front.setPower(speed / 2.0);
                    rightMotor_front.setPower(speed / 2.0);
                    telemetry.addData("Motor Output", df.format(speed) + ", " +
                            df.format(speed));
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(limit(speed + output));
                    rightMotor.setPower(limit(speed - output));
                    leftMotor_front.setPower(limit((speed + output) / 2.0));
                    rightMotor_front.setPower(limit((speed - output) / 2.0));
                    telemetry.addData("Motor Output", df.format(limit(speed + output)) + ", " +
                            df.format(limit(speed - output)));
                }
            } else {
                /* No sensor update has been received since the last time  */
                /* the loop() function was invoked.  Therefore, there's no */
                /* need to update the motors at this time.                 */
            }
            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            telemetry.update();

        }





        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);
        rightMotor_front.setPower(0.0);
        leftMotor_front.setPower(0.0);


        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void turnToDegrees(double degrees, boolean rightSidePower) {

        yawPIDController.enable(false);

        yawPIDController.setSetpoint(degrees);

        yawPIDController.enable(true);

        yawPIDResult = new navXPIDController.PIDResult();

        while (!yawPIDResult.isOnTarget()) {

            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                double output = yawPIDResult.getOutput();
                leftMotor.setPower(output);
                leftMotor_front.setPower(output);

                // This is only used for the right column, since the robot has to turn sharper
                // in order to make it in the cryptobox
                if (rightSidePower) {
                    rightMotor_front.setPower(-output);
                    rightMotor.setPower(-output);
                }

                telemetry.addData("Motor Output", df.format(output) + ", " +
                        df.format(-output));
                telemetry.update();

                // Move on if the robot gets stuck trying to turn w/ 4 sec left
                if (totalRuntime.time() > 26.0) {
                    leftMotor.setPower(0.0);
                    leftMotor_front.setPower(0.0);
                    rightMotor.setPower(0.0);
                    rightMotor_front.setPower(0.0);
                    return;
                }
            }

        }

        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);
        rightMotor_front.setPower(0.0);
        leftMotor_front.setPower(0.0);

        /*
        I think this is why it was not working for center column
        yesterday, I was only setting 0 power to the leftMotor
        and not any of the other ones. So the others kept moving since they
        were all moving after the first -90 degree turn.
         */

    }

    public void raiseElevator() {
        runtime.reset();
        while (runtime.time() < 0.7) {
            elevator.setPower(1); // 0.3
        }
        elevator.setPower(0.0);
    }

    public void lowerElevator() {
        runtime.reset();
        while (runtime.time() < 0.4) {
            elevator.setPower(-0.95); // -0.25
        }
        elevator.setPower(0.0);
    }

    public void backUp(double speed, double inches) {

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor_front.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor_front.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (inches * 114.65);

        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (leftMotor.isBusy() && rightMotor.isBusy()) {

            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    leftMotor.setPower(speed);
                    rightMotor.setPower(speed);
                    leftMotor_front.setPower(speed / 2.0);
                    rightMotor_front.setPower(speed / 2.0);
                    telemetry.addData("Motor Output", df.format(speed) + ", " +
                            df.format(speed));
                } else {
                    double output = yawPIDResult.getOutput();
                    leftMotor.setPower(limit(speed + output));
                    rightMotor.setPower(limit(speed - output));
                    leftMotor_front.setPower(limit((speed + output) / 2.0));
                    rightMotor_front.setPower(limit(speed - output) / 2.0);
                    telemetry.addData("Motor Output", df.format(limit(speed + output)) + ", " +
                            df.format(limit(speed - output)));
                }
            } else {
                /* No sensor update has been received since the last time  */
                /* the loop() function was invoked.  Therefore, there's no */
                /* need to update the motors at this time.                 */
            }
            telemetry.addData("Yaw", df.format(navx_device.getYaw()));
            telemetry.update();

        }
        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);
        rightMotor_front.setPower(0.0);
        leftMotor_front.setPower(0.0);


        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor_front.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor_front.setDirection(DcMotor.Direction.REVERSE);

    }
}
