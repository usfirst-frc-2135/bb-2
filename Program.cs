using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.LED;
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
    private const int ThreadLoopTime = 10;  // loop time in msec
    private const int SignalLightMax = 50;  // number of loops for toggling RSL

    // Constants - Create gamepad instance
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

    private const int NumButtons = 12;
    private const int NumAxes = 6;

    // NOTE: DPAD does not work as separate buttons!

    // Constants - PCM ports
    private const int PcmPortSignalLight = 7;   // Robot signal light port
    private const int NumValves = 6;

    // Constants - Joysticks
    private const float Deadband = 0.10F;   // Joystick deadband for driving

    // Constants - CANDle settings
    private const float Brightness = 0.5F; // CANDle brightness level
    private const int NumLeds = 8;         // CANDle total number of LEDs
    private const int OffsetLed = 0;       // CANDle offset of first LED
    private const float Speed = 0.25F;     // CANDle animation speed
    private const int White = 0;           // CANDle white level
    private const int LedPeriodMs = 500;   // LED flashing period in msec

    // Static objects
    private static readonly LogitechGamepad _gamepad = new LogitechGamepad(UsbHostDevice.GetInstance(), 0);

    // Create four drive talons and PCM on CAN bus
    private static readonly TalonSRX _leftFrnt = new TalonSRX(1);
    private static readonly TalonSRX _leftRear = new TalonSRX(2);
    private static readonly TalonSRX _rghtFrnt = new TalonSRX(3);
    private static readonly TalonSRX _rghtRear = new TalonSRX(4);
    private static readonly TalonSRX _wrist = new TalonSRX(6);
    private static readonly CANdle _candle = new CANdle(0);
    private static readonly PneumaticControlModule _pcm = new PneumaticControlModule(0);

    // Global variables
    private static bool _enabled = false;
    private static DateTime _enabledTime;
    private static Animation[] _animation = {
            null,
            new ColorFlowAnimation(255, 48, 0, White, Speed, NumLeds, ColorFlowAnimation.ColorFlowDirection.Forward, OffsetLed),
            new FireAnimation(Brightness, Speed, NumLeds, 1, 1, false, OffsetLed),
            new LarsonAnimation(128, 128, 128, White, Speed, NumLeds, LarsonAnimation.LarsonBounceMode.Front, 2, OffsetLed),
            new RainbowAnimation(Brightness, Speed, NumLeds, false, OffsetLed),
            new RgbFadeAnimation(Brightness, Speed, NumLeds, OffsetLed),
            new SingleFadeAnimation(255, 48, 0, White, Speed, NumLeds, OffsetLed),
            new StrobeAnimation(255, 48, 0, White, Speed, NumLeds, OffsetLed),
            new TwinkleAnimation(255, 48, 0, White, Speed, NumLeds, TwinkleAnimation.TwinklePercent.Percent64, OffsetLed),
            new TwinkleOffAnimation(255, 48, 0, White, Speed, NumLeds, TwinkleOffAnimation.TwinkleOffPercent.Percent64, OffsetLed)
        };
    private static int _activeAnimation = 0;

    //*********************************************************************
    //*********************************************************************
    //
    //  Initialization
    //
    private static void configDrive()
    {
      // Invert all motor directions to match installation
      _rghtFrnt.SetInverted(true);
      _rghtRear.SetInverted(true);
      _leftFrnt.SetInverted(true);
      _leftRear.SetInverted(true);
    }

    private static void configWrist()
    {
      // TODO: configure Talon for Motion Magic on wrist (need gear ratio)
      _wrist.SetInverted(false);
    }

    private static void ConfigCANdle()
    {
      _candle.ConfigFactoryDefault();

      CANdleConfiguration configAll = new CANdleConfiguration();

      configAll.brightnessScalar = Brightness;
      configAll.disableWhenLOS = false;
      configAll.statusLedOffWhenActive = true;
      configAll.stripType = LEDStripType.RGB;
      configAll.v5Enabled = true;
      configAll.vBatOutputMode = VBatOutputMode.Off;
      _candle.ConfigAllSettings(configAll, 100);

      _candle.ClearAnimation(0);

      // _candle.Animate(_animation[_activeAnimation]);
      _candle.SetLEDs(255, 48, 0);
    }

    private static void ConfigValves()
    {
      HandleValvePulse(0, false);
      HandleValvePulse(1, false);
      HandleValvePulse(2, false);
      HandleValvePulse(3, false);
      HandleValvePulse(4, false);
      HandleValvePulse(5, false);
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Deadband the joystick input
    //      If value is within +/-deadband range of center, clear it.
    //
    private static void StickDeadband(ref double value)
    {
      if (value < -Deadband)
        value = (value + Deadband) / (1.0 - Deadband);  /* outside of deadband */
      else if (value > +Deadband)
        value = (value - Deadband) / (1.0 - Deadband);  /* outside of deadband */
      else
        value = 0;                                      /* within deadband so zero it */
    }

    //*********************************************************************
    //
    //  Nomalize the vector sum of mecanum math.
    //      Some prefer to  scale from the max possible value to '1'.
    //      Others prefer to simply cut off if the sum exceeds '1'.
    //
    private static void DriveNormalize(ref double toNormalize)
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
      double turn = _gamepad.GetAxis(AxisRightStickX);  // right x: Positive is turn-right, negative is turn-left

      if (!_enabled)
      {
        x = 0.0;
        y = 0.0;
        turn = 0.0;
      }

      StickDeadband(ref x);
      StickDeadband(ref y);
      StickDeadband(ref turn);

      double _leftFrnt_throt = y + x + turn;   // left front moves positive for forward, strafe-right, turn-right
      double _leftRear_throt = y - x + turn;   // left rear moves positive for forward, strafe-left, turn-right
      double _rghtFrnt_throt = y - x - turn;   // right front moves positive for forward, strafe-left, turn-left
      double _rghtRear_throt = y + x - turn;   // right rear moves positive for forward, strafe-right, turn-left

      // normalize here, there a many way to accomplish this, this is a simple solution
      DriveNormalize(ref _leftFrnt_throt);
      DriveNormalize(ref _leftRear_throt);
      DriveNormalize(ref _rghtFrnt_throt);
      DriveNormalize(ref _rghtRear_throt);

      // Control the motors for mecanum drive operation
      _leftFrnt.Set(ControlMode.PercentOutput, _leftFrnt_throt);
      _leftRear.Set(ControlMode.PercentOutput, _leftRear_throt);
      _rghtFrnt.Set(ControlMode.PercentOutput, _rghtFrnt_throt);
      _rghtRear.Set(ControlMode.PercentOutput, _rghtRear_throt);
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Detect button pressed - return once when first depressed
    //
    public static bool[] btnSave = new bool[NumButtons] { false, false, false, false, false, false, false, false, false, false, false, false };

    private static bool IsButtonPressed(uint buttonIdx)
    {
      bool result = false;
      bool btnState = _gamepad.GetButton(buttonIdx);

      if (btnState && !btnSave[buttonIdx])
        result = true;

      btnSave[buttonIdx] = btnState;
      return result;
    }

    //*********************************************************************
    //
    //  Detect start button and use it to enable the robot
    //
    private static void HandleEnableButton()
    {
      // Enable button is pressed, enable and capture start time
      if (IsButtonPressed(BtnStart))
      {
        _enabled = !_enabled;
        if (_enabled)
          _enabledTime = DateTime.Now;
        Debug.Print("BB-2 is now: " + ((_enabled) ? "ENABLED" : "DISABLED") + " at " + DateTime.Now + " seconds");
      }
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

      if (IsButtonPressed(BtnLeftBumper))
      {
        _candle.ClearAnimation(0);
        _candle.SetLEDs(0, 0, 0, 0, OffsetLed, NumLeds);
        _activeAnimation += 1;
        if (_activeAnimation >= _animation.Length)
          _activeAnimation = 0;
        if (_animation[_activeAnimation] != null)
          _candle.Animate(_animation[_activeAnimation]);
      }
      else if (IsButtonPressed(BtnLeftTrigger))
      {
        _candle.ClearAnimation(0);
        _candle.SetLEDs(0, 0, 0, 0, OffsetLed, NumLeds);
        _activeAnimation -= 1;
        if (_activeAnimation < 0)
          _activeAnimation = _animation.Length - 1;
        if (_animation[_activeAnimation] != null)
          _candle.Animate(_animation[_activeAnimation]);
      }
    }

    //*********************************************************************
    //
    //  Handle Valve pulse timing
    //
    public static DateTime[] openTime = new DateTime[NumValves] { DateTime.MinValue, DateTime.MinValue, DateTime.MinValue, DateTime.MinValue, DateTime.MinValue, DateTime.MinValue };

    private static bool HandleValvePulse(int valveIdx, bool startPulse)
    {
      bool valveOpen = false;

      if ((openTime[valveIdx] == DateTime.MinValue) && startPulse)
        openTime[valveIdx] = DateTime.Now;

      if ((openTime[valveIdx] != DateTime.MinValue) && (DateTime.Now.Subtract(openTime[valveIdx]).Milliseconds < 250))
        valveOpen = true;
      else
        openTime[valveIdx] = DateTime.MinValue;

      _pcm.SetSolenoidOutput(valveIdx, valveOpen);
      return valveOpen;
    }

    //*********************************************************************
    //
    //   Fire PCM solenoids based on button input
    //
    private static void HandleFiringButtons()
    {
      HandleValvePulse(0, IsButtonPressed(BtnX));
      HandleValvePulse(1, IsButtonPressed(BtnA));
      HandleValvePulse(2, IsButtonPressed(BtnB));
      HandleValvePulse(3, IsButtonPressed(BtnY));
      HandleValvePulse(4, IsButtonPressed(BtnRightBumper));
      HandleValvePulse(5, IsButtonPressed(BtnRightTrigger));
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Process enabled state operation of robot
    //
    private static void SetSignalLight(bool onState)
    {
      _pcm.SetSolenoidOutput(PcmPortSignalLight, onState);
      if (onState)
        _candle.SetLEDs(255, 48, 0);
      else
        _candle.SetLEDs(0, 0, 0);
    }

    private static void HandleEnabledState()
    {
      if (_gamepad.GetConnectionStatus() != CTRE.Phoenix.UsbDeviceConnection.Connected)
        _enabled = false;

      if (_enabled)
      {
        // this lets the drive motors, wrist motor, and PCM take commands
        Watchdog.Feed();

        TimeSpan onTime = DateTime.Now.Subtract(_enabledTime);

        if (onTime.Seconds > 180)
          _enabled = false;

        if (_animation[_activeAnimation] == null)
          if (onTime.Milliseconds > LedPeriodMs)
            SetSignalLight(true);
          else
            SetSignalLight(false);
      }
      else
        SetSignalLight(true);
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Debug gamepad axes and buttons
    //
    private static int _debugPrintCount = 0;

    private static void DebugController()
    {
      if (++_debugPrintCount % 50 == 0)
      {
        GameControllerValues gpadValues = new GameControllerValues();
        _gamepad.GetAllValues(ref gpadValues);
        float[] a;
        uint btns;
        int pov;

        a = gpadValues.axes;
        btns = gpadValues.btns;
        pov = gpadValues.pov;

        //Debug.Print("a0:" + a[0] + " a1:" + a[1] + " a2:" + a[2] + " a3:" + a[3] + " a4:" + a[4] + " a5:" + a[5] +
        //    " btns:" + btns + " pov:" + pov);

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
        //    Debug.Print("a0:" + axis0 + " a1:" + axis1 + " a2:" + axis2 + " a3:" + axis3 + " a4:" + axis4 + " a5:" + axis5 +
        //                " b1:" + btn1 + " b2:" + btn2 + " b3:" + btn3 + " b4:" + btn4 + " b5:" + btn5 +
        //                " b6:" + btn6 + " b7:" + btn7 + " b8:" + btn8 + " b9:" + btn9 + " b10:" + btn10 +
        //                " b11:" + btn11 + " b12:" + btn12 + " pov:" + pov0);
      }
    }

    //*********************************************************************
    //
    //  Main program
    //
    public static void Main()
    {
      // Get the start time for tracking total on time
      configDrive();
      configWrist();
      ConfigCANdle();
      ConfigValves();

      // TODO: Initialize PCM and enable compressor (if needed) Compressor may be automatic

      //
      // Main loop (forever)
      //
      while (true)
      {
        HandleDrive();                  // use joysticks to run drive motors
        HandleEnableButton();           // handle start button to enable and disable robot
        HandleWristButtons();           // handle buttons that control wrist elevation
        HandleFiringButtons();          // handle buttons that control the firing solenoids
        HandleEnabledState();           // check enable state and process it
        DebugController();              // temporary logging code for printing inputs

        Thread.Sleep(ThreadLoopTime);   // this provides the loop pacing
      }
    }
  }
}
