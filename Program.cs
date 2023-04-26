using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.Threading;

//*****************************************************************************
//
//  BB-2: Presentation Invasion 2135 - T-shirt shooter
//
namespace BB_2
{
    public class Program
    {
        // Constants
        private const int ThreadLoopTime = 20;  // loop time in msec
        private const int SignalLightMax = 25;  // number of loops for toggling RSL

        // Create gamepad instance
        private const int BtnX = 1;             // X button             - fire
        private const int BtnA = 2;             // A button             - fire
        private const int BtnB = 3;             // B button             - fire
        private const int BtnY = 4;             // Y button             - fire
        private const int BtnLeftBumper = 5;    // Left Bumper          - wrist up
        private const int BtnLeftTrigger = 7;   // Left Trigger         - wrist down
        private const int BtnRightBumper = 6;   // Right Bumper         - fire
        private const int BtnRightTrigger = 8;  // Right Trigger        - fire
        // private const int BtnBack = 9;          // Back button          - unused
        private const int BtnStart = 10;        // Start button         - enable robot
        // private const int BtnLeftStick = 11;    // Left joystick press  - unused
        // private const int BtnRightStick = 12;   // Right joystick press - unused

        private const int AxisLeftStickX = 0;   // Left joystick X direction    - strafe
        private const int AxisLeftStickY = 1;   // Left joystick Y direction    - forward/reverse
        private const int AxisRightStickX = 2;  // Right joystick X direction   - rotation
        // private const int AxisRightStickY = 5;  // Right joystick Y direction   - unused

        // NOTE: DPAD does not work as separate buttons!

        // Constants - PCM ports
        private const int PcmPortSignalLight = 7;   // Robot signal light port

        // Static objects
        private static readonly LogitechGamepad _gamepad = new LogitechGamepad(UsbHostDevice.GetInstance(), 0);

        // Create four drive talons and PCM on CAN bus
        private static readonly TalonSRX _leftFrnt = new TalonSRX(1);
        private static readonly TalonSRX _leftRear = new TalonSRX(2);
        private static readonly TalonSRX _rghtFrnt = new TalonSRX(3);
        private static readonly TalonSRX _rghtRear = new TalonSRX(4);

        private static readonly TalonSRX _wrist = new TalonSRX(6);

        private static readonly PneumaticControlModule _pcm = new PneumaticControlModule(0);

        // Global variables
        private static bool _enabled = false;
        private static bool _enableBtnWasDown = false;
        private static DateTime _startTime;
        private static DateTime _enabledTime;
        private static int _signalLightCount = 0;

        //*********************************************************************
        //
        //  Main program
        //
        public static void Main()
        {
            // Get the start time for tracking total on time
            _startTime = DateTime.Now;

            // Invert all motor directions to match installation
            _rghtFrnt.SetInverted(true);
            _rghtRear.SetInverted(true);
            _leftFrnt.SetInverted(true);
            _leftRear.SetInverted(true);

            // TODO: Initialize PCM and enable compressor (if needed) Compressor may be automatic

            //
            // Main loop (forever)
            //
            while (true)
            {
                HandleEnabledState();           // check enable state and process it

                HandleEnableButton();           // handle start button to enable and disable robot

                HandleDrive();                  // use joysticks to run drive motors

                HandleWristButtons();           // handle buttons that control wrist elevation

                HandleFiringButtons();          // handle buttons that control the firing solenoids

                //DebugController();              // temporary logging code for printing inputs

                Thread.Sleep(ThreadLoopTime);   // this provides the loop pacing at 20msec
            }
        }

        //*********************************************************************
        //
        //  Process enabled state operation of robot
        //
        private static void SetSignalLight(bool onState)
        {
            _pcm.SetSolenoidOutput(PcmPortSignalLight, onState);
        }

        private static void HandleEnabledState()
        {
            if (_gamepad.GetConnectionStatus() != CTRE.Phoenix.UsbDeviceConnection.Connected)
                _enabled = false;

            // keep feeding watchdog to enable motors
            if (_enabled)
            {
                // this lets the drive motors, wrist motor, and PCM take commands
                Watchdog.Feed();

                // TODO: time out and disable after 3 minutes
                //  if (DateTime.Now.Ticks - _enabledTime.Ticks > 180)
                //      _enabled = false;

                if (_signalLightCount++ >= SignalLightMax)
                {
                    if (_pcm.GetSolenoidOutput(PcmPortSignalLight))
                        SetSignalLight(false);
                    else
                        SetSignalLight(true);
                    _signalLightCount = 0;
                }
            }
            else
                SetSignalLight(true);
        }

        //*********************************************************************
        //
        //  Get gamepad buttons and handle actions needed
        //
        private static void HandleEnableButton()
        {
            // Enable button is pressed, enable and capture start time
            if (_gamepad.GetButton(BtnStart))
                _enableBtnWasDown = true;
            else if (_enableBtnWasDown)
            {
                _enabled = !_enabled;
                _enableBtnWasDown = false;
                if (_enabled)
                {
                    _signalLightCount = 0;
                    _enabledTime = DateTime.Now;
                }
                Debug.Print("BB-2 is now: " + ((_enabled) ? "ENABLED" : "DISABLED") + " at " + DateTime.Now);
            }
        }

        //*********************************************************************
        //
        //  Deadband the joystick input
        //      If value is within +/-deadband range of center, clear it.
        //
        private static void Deadband(ref double value)
        {
            double deadband = 0.10;

            if (value < -deadband)
                value = (value + deadband) / (1.0 - deadband);  /* outside of deadband */
            else if (value > +deadband)
                value = (value - deadband) / (1.0 - deadband);  /* outside of deadband */
            else
                value = 0;                                      /* within deadband so zero it */
        }

        //*********************************************************************
        //
        //  Nomalize the vector sum of mecanum math.
        //      Some prefer to  scale from the max possible value to '1'.
        //      Others prefer to simply cut off if the sum exceeds '1'.
        //
        private static void Normalize(ref double toNormalize)
        {
            if (toNormalize > 1)
                toNormalize = 1;
            else if (toNormalize < -1)
                toNormalize = -1;
            else
            { /* nothing to do */ }
        }

        //*********************************************************************
        //
        //  Run drive motors from joystick inputs
        //      X axis left - strafe left/right
        //      Y axis left - forward/back (joystick axis Y is negative forward)
        //      X axis right - turn left/right
        //
        private static void HandleDrive()
        {
            double x = _gamepad.GetAxis(AxisLeftStickX);      // left x: Positive is strafe-right, negative is strafe-left
            double y = -1 * _gamepad.GetAxis(AxisLeftStickY); // left y: Positive is forward, negative is reverse
            double turn = _gamepad.GetAxis(AxisRightStickX);     // right x: Positive is turn-right, negative is turn-left

            if (!_enabled)
            {
                x = 0.0;
                y = 0.0;
                turn = 0.0;
            }

            Deadband(ref x);
            Deadband(ref y);
            Deadband(ref turn);

            double _leftFrnt_throt = y + x + turn;   // left front moves positive for forward, strafe-right, turn-right
            double _leftRear_throt = y - x + turn;   // left rear moves positive for forward, strafe-left, turn-right
            double _rghtFrnt_throt = y - x - turn;   // right front moves positive for forward, strafe-left, turn-left
            double _rghtRear_throt = y + x - turn;   // right rear moves positive for forward, strafe-right, turn-left

            // normalize here, there a many way to accomplish this, this is a simple solution
            Normalize(ref _leftFrnt_throt);
            Normalize(ref _leftRear_throt);
            Normalize(ref _rghtFrnt_throt);
            Normalize(ref _rghtRear_throt);

            // Control the motors for mecanum drive operation
            _leftFrnt.Set(ControlMode.PercentOutput, _leftFrnt_throt);
            _leftRear.Set(ControlMode.PercentOutput, _leftRear_throt);
            _rghtFrnt.Set(ControlMode.PercentOutput, _rghtFrnt_throt);
            _rghtRear.Set(ControlMode.PercentOutput, _rghtRear_throt);
        }

        //*********************************************************************
        //
        //  Get button input and operate wrist
        //
        private static void HandleWristButtons()
        {
            // Use wrist buttons for up and down to control wrist elevation
            if (_gamepad.GetButton(BtnLeftBumper))
            {
                //_wrist.Set(ControlMode.PercentOutput, 0.5);
            }
            else if (_gamepad.GetButton(BtnLeftTrigger))
            {
                //_wrist.Set(ControlMode.PercentOutput, -0.5);
            }
            else
            {
                _wrist.Set(TalonSRXControlMode.PercentOutput, 0);
            }
        }

        private static void HandleFiringButtons()
        {
            // Fire PCM solenoids based on button input
            // TODO: Create X msec pulse when a button is pressed
            _pcm.SetSolenoidOutput(0, _gamepad.GetButton(BtnX));            // solenoid valve 1
            _pcm.SetSolenoidOutput(1, _gamepad.GetButton(BtnA));            // solenoid valve 2
            _pcm.SetSolenoidOutput(2, _gamepad.GetButton(BtnB));            // solenoid valve 3
            _pcm.SetSolenoidOutput(3, _gamepad.GetButton(BtnY));            // solenoid valve 4
            _pcm.SetSolenoidOutput(4, _gamepad.GetButton(BtnRightBumper));  // solenoid valve 5
            _pcm.SetSolenoidOutput(5, _gamepad.GetButton(BtnRightTrigger)); // solenoid valve 6
        }

        //*********************************************************************
        //
        //  Debug gamepad axes and buttons
        //
        private static int _debugPrintCount = 0;

        private static void DebugController()
        {
            // Get all axis and buttons
            double axis0 = _gamepad.GetAxis(0);      // left jstick x -1.0 to 1.0
            double axis1 = _gamepad.GetAxis(1);      // left jstick y 1.0 to -1.0
            double axis2 = _gamepad.GetAxis(2);      // right jstick x -1.0 to 1.0
            double axis3 = _gamepad.GetAxis(3);      // (doesn't work)
            double axis4 = _gamepad.GetAxis(4);      // (doesn't work)
            double axis5 = _gamepad.GetAxis(5);      // right jstick y 1.0 to -1.0

            bool btn1 = _gamepad.GetButton(1);       // x
            bool btn2 = _gamepad.GetButton(2);       // a
            bool btn3 = _gamepad.GetButton(3);       // b
            bool btn4 = _gamepad.GetButton(4);       // y
            bool btn5 = _gamepad.GetButton(5);       // left bumper
            bool btn6 = _gamepad.GetButton(6);       // right bumper
            bool btn7 = _gamepad.GetButton(7);       // left trigger
            bool btn8 = _gamepad.GetButton(8);       // right trigger
            bool btn9 = _gamepad.GetButton(9);       // back
            bool btn10 = _gamepad.GetButton(10);     // start
            bool btn11 = _gamepad.GetButton(11);     // left jstick
            bool btn12 = _gamepad.GetButton(12);     // right jstick
            int pov0 = _gamepad.GetPov();            // POV (doesn't work!)

            // Print to console so we can debug them
            if (++_debugPrintCount % 25 == 0)
            {
                Debug.Print("a0:" + axis0 + " a1:" + axis1 + " a2:" + axis2 + " a3:" + axis3 + " a4:" + axis4 + " a5:" + axis5 +
                            " b1:" + btn1 + " b2:" + btn2 + " b3:" + btn3 + " b4:" + btn4 + " b5:" + btn5 +
                            " b6:" + btn6 + " b7:" + btn7 + " b8:" + btn8 + " b9:" + btn9 + " b10:" + btn10 + 
                            " b11:" + btn11 + " b12:" + btn12 + " pov:" + pov0);
            }
        }
    }
}
