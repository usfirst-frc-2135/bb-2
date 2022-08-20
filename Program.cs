using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.Text;
using System.Threading;

namespace BB_2
{
    public class Program
    {
        /* create four talons and PCM */
        private static TalonSRX leftFrnt = new TalonSRX(1);
        private static TalonSRX leftRear = new TalonSRX(2);
        private static TalonSRX rghtFrnt = new TalonSRX(3);
        private static TalonSRX rghtRear = new TalonSRX(4);
        private static TalonSRX wrist = new TalonSRX(6);
        private static PneumaticControlModule _pcm = new PneumaticControlModule(0);

        private static GameController _gamepad = null;

        // global variables
        private static bool enabled = false;
        private static bool enableBtnDown = false;
        private static DateTime starttime;
        private static DateTime enabletime;

        public static void Main()
        {
            // Get the start time for tracking total on time
            starttime = DateTime.Now;

            // Create the gamepad object
            if (null == _gamepad)
                _gamepad = new GameController(UsbHostDevice.GetInstance());

            // Invert the right hand motor directions
            rghtFrnt.SetInverted(true);
            rghtRear.SetInverted(true);

            // Initialize PCM and enable compressor
            // Compressor may be automatic

            // loop forever
            while (true)
            {
                // keep feeding watchdog to enable motors
                if (enabled)
                    Watchdog.Feed();

                // use joysticks to run drive motors
                Drive();

                // use a joystick axis or dPad to move the wrist
                HandleWrist();

                // read buttons and control the firing solenoids
                HandleButtons();

                // temporary logging code for printing inputs
                DebugController();

                // this provides the loop pacing at 20msec
                Thread.Sleep(20);
            }
        }

        /**
         * If value is within 10% of center, clear it.
         * @param [out] floating point value to deadband.
         */
        private static void Deadband(ref float value)
        {
            if (value < -0.10)
            { /* outside of deadband */ }
            else if (value > +0.10)
            { /* outside of deadband */ }
            else
            { /* within 10% so zero it */
                value = 0;
            }
        }

        /**
         * Nomalize the vector sum of mecanum math.  Some prefer to 
         * scale from the max possible value to '1'.  Others
         * prefer to simply cut off if the sum exceeds '1'.
         */
        private static void Normalize(ref float toNormalize)
        {
            if (toNormalize > 1)
                toNormalize = 1;
            else if (toNormalize < -1)
                toNormalize = -1;
            else
                { /* nothing to do */ }
        }

        // run drive motors from joystick inputs
        //  X axis left - strafe left/right
        //  Y axis left - forward/back (joystick axis Y is negative forward)
        //  X axis right - turn left/right
        private static void Drive()
        {
            float x    = _gamepad.GetAxis(0);      // Positive is strafe-right, negative is strafe-left
            float y    = -1 * _gamepad.GetAxis(1); // Positive is forward, negative is reverse
            float turn = _gamepad.GetAxis(2);  // Positive is turn-right, negative is turn-left

            Deadband(ref x);
            Deadband(ref y);
            Deadband(ref turn);

            float leftFrnt_throt = y + x + turn; // left front moves positive for forward, strafe-right, turn-right
            float leftRear_throt = y - x + turn; // left rear moves positive for forward, strafe-left, turn-right
            float rghtFrnt_throt = y - x - turn; // right front moves positive for forward, strafe-left, turn-left
            float rghtRear_throt = y + x - turn; // right rear moves positive for forward, strafe-right, turn-left

            /* normalize here, there a many way to accomplish this, this is a simple solution */
            Normalize(ref leftFrnt_throt);
            Normalize(ref leftRear_throt);
            Normalize(ref rghtFrnt_throt);
            Normalize(ref rghtRear_throt);

            /* everything up until this point assumes positive spins motor so that robot moves forward.
                But typically one side of the robot has to drive negative (red LED) to move robor forward.
                Assuming the left-side has to be negative to move robot forward, flip the left side */
            /* This can be removed if the calls to SetInverted work as intended */
            // rghtFrnt_throt *= -1;
            // rghtRear_throt *= -1;

            leftFrnt.Set(ControlMode.PercentOutput, leftFrnt_throt);
            leftRear.Set(ControlMode.PercentOutput, leftRear_throt);
            rghtFrnt.Set(ControlMode.PercentOutput, rghtFrnt_throt);
            rghtRear.Set(ControlMode.PercentOutput, rghtRear_throt);
        }

        private static void HandleWrist()
        {
            // Get wrist axis or dPad from gamepad
            // _gamepad.GetAxis();
            // derive a intended wrist angle

            // Move wrist motor up/down
            // wrist.Set(TalonSRXControlMode.Position, wrist_angle);
        }

        private static void HandleButtons()
        {
            // Get the shooting buttons
            // Fire PCM solenoids based on button input
            bool btn0 = _gamepad.GetButton(0);
            bool btn1 = _gamepad.GetButton(1);
            bool btn2 = _gamepad.GetButton(2);
            bool btn3 = _gamepad.GetButton(3);
            bool btn4 = _gamepad.GetButton(4);
            bool btn5 = _gamepad.GetButton(5);
            bool btn6 = _gamepad.GetButton(6);
            bool btn7 = _gamepad.GetButton(7);

            _pcm.SetSolenoidOutput(0, btn0);
            _pcm.SetSolenoidOutput(1, btn1);
            _pcm.SetSolenoidOutput(2, btn2);
            _pcm.SetSolenoidOutput(3, btn3);
            _pcm.SetSolenoidOutput(4, btn4);
            _pcm.SetSolenoidOutput(5, btn5);

            // Enable button is pressed, enable and capture start time
            bool enableBtn = btn0;
            if (enableBtn)
                enableBtnDown = false;
            else if (!enableBtnDown)
            {
                enabled = !enabled;
                if (enabled)
                    enabletime = DateTime.Now;
            }

            // If enabled, time out and disable after 3 minutes
            if (enabled)
            {
                if (DateTime.Now.Ticks - enabletime.Ticks > 180)
                    enabled = false;
            }
        }

        private static void DebugController()
        {
            // Get all axis and buttons
            float axis0 = _gamepad.GetAxis(0);
            float axis1 = _gamepad.GetAxis(1);
            float axis2 = _gamepad.GetAxis(2);
            float axis3 = _gamepad.GetAxis(3);
            float axis4 = _gamepad.GetAxis(4);
            float axis5 = _gamepad.GetAxis(5);

            bool btn0 = _gamepad.GetButton(0);
            bool btn1 = _gamepad.GetButton(1);
            bool btn2 = _gamepad.GetButton(2);
            bool btn3 = _gamepad.GetButton(3);
            bool btn4 = _gamepad.GetButton(4);
            bool btn5 = _gamepad.GetButton(5);
            bool btn6 = _gamepad.GetButton(6);
            bool btn7 = _gamepad.GetButton(7);
            bool btn8 = _gamepad.GetButton(8);
            bool btn9 = _gamepad.GetButton(9);
            bool btn10 = _gamepad.GetButton(10);
            bool btn11 = _gamepad.GetButton(11);

            // Print to console so we can debug them
            Debug.Print("a0: " + axis0 + " a1:" + axis1 + " a2:" + axis2 +
                " a3:" + axis3 + " a4:" + axis4 + " a5:" + axis5 +
                " b0:" + btn0 + " b1:" + btn1 + " b2:" + btn2 + " b3:" + btn3 +
                " b4:" + btn4 + " b5:" + btn5 + " b6:" + btn6 + " b7:" + btn7 +
                " b8:" + btn8 + " b9:" + btn9 + " b10:" + btn10 + " b11:" + btn11);
        }
    }
}