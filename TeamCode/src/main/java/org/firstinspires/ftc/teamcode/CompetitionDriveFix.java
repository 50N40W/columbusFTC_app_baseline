package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by howard on 11/4/17.
 */


//package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 */

@TeleOp(name="Time Slide Op Mode", group="Pushbot")
//@Disabled
public class CompetitionDriveFix extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        ElapsedTime runtime = new ElapsedTime();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        double leftPower = 0;
        double rightPower = 0;


        //A Timing System By Jeffrey & Alexis
        // long currentThreadTimeMillis (0);
        //

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // ****************************************************************
        //  Post Initialization

        /***************************************************************************
         *            Everything below here happens after we press START           *
         ***************************************************************************/

        //@Override
        final long BASEFRAME = 30;
        final long SENSORPERIOD     = BASEFRAME;
        final long ENCODERPERIOD    = BASEFRAME;
        final long SERVOPERIOD      = BASEFRAME;
        final long NAVPERIOD        = BASEFRAME;
        final long MOTORPERIOD      = BASEFRAME;
        final long CONTROLLERPERIOD = BASEFRAME;
        final long TELEMETRYPERIOD = 1000;

        long CurrentTime=System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime+5;
        long LastServo = CurrentTime+10;
        long LastNav = CurrentTime+15;
        long LastMotor = CurrentTime+20;
        long LastController = CurrentTime+25;
        long LastTelemetry = CurrentTime+17;

        //int overRun1 = 0;
        //int overRun2 = 0;
        //int skipped0 = 0;
        //int skipped1 = 0;
        //int skipped2 = 0;
        //int accumTurn = 0;

        //double legTime = CurrentTime;
        double lastTelemetry = CurrentTime;
        double timeLeft = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        OpticalDistanceSensor odsSensor1;  // Hardware Device Object
        OpticalDistanceSensor odsSensor2;  // Hardware Device Object
        odsSensor1 = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");
        odsSensor2 = hardwareMap.get(OpticalDistanceSensor.class, "sensor2_ods");



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double drive = 0;
            double turn = 0;
            CurrentTime = System.currentTimeMillis();

            //Loop For Timing System
            /********************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                double light1 = odsSensor1.getLightDetected();
                double light2 = odsSensor2.getLightDetected();
            }

            /********************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;
            }

            /********************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
            }

            /********************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;
                turn = turn*turn*turn; // raise to 3rd power to make it smoother
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            }

            /********************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD){
                LastMotor = CurrentTime;
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
            }

            /********************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD){
                LastController = CurrentTime;
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.right_stick_x;


            }

            /********************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.update();
            }
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}