using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.ComponentModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Text.RegularExpressions;
using x_IMU_API;

namespace x_IMU_IMU_and_AHRS_Algorithms
{
    class Program
    {
        /// <summary>
        /// Algorithm object.
        /// </summary>
        static AHRS.MadgwickAHRS AHRS = new AHRS.MadgwickAHRS(1f / 1024f, 50f);
       // static AHRS.MahonyAHRS AHRS = new AHRS.MahonyAHRS(1f / 256f, 5f);
        private static Form_3Dcuboid _form3DcuboidB;
        private static Stopwatch _stopWatch = new Stopwatch();
        static readonly BackgroundWorker _backgroundWorkerB = new BackgroundWorker();
        /// <summary>
        /// Main method.
        /// </summary>
        private static void Main()
        {
            Console.WriteLine(Assembly.GetExecutingAssembly().GetName().Name + " " +
                              Assembly.GetExecutingAssembly().GetName().Version.Major + "." +
                              Assembly.GetExecutingAssembly().GetName().Version.Minor);

            try
            {
                var xImUserial = new QuadRotorImu(_stopWatch);
                xImUserial.Open();
                // Show 3D cuboid forms
                Console.WriteLine("Showing 3D Cuboid forms...");
                _form3DcuboidB =
                    new Form_3Dcuboid(new[]
                    {
                        "Form_3Dcuboid/RightInv.png", "Form_3Dcuboid/LeftInv.png", "Form_3Dcuboid/BackInv.png",
                        "Form_3Dcuboid/FrontInv.png", "Form_3Dcuboid/TopInv.png", "Form_3Dcuboid/BottomInv.png"
                    });
                _form3DcuboidB.Text += " B";

                _backgroundWorkerB.DoWork += delegate
                {
                    _form3DcuboidB.ShowDialog();
                };
                _backgroundWorkerB.RunWorkerAsync();

                // Algorithm uses AHRS update method.
                Console.WriteLine("Algorithm running in AHRS mode.");
                xImUserial.CalInertialAndMagneticDataReceived += xIMUserial_CalInertialAndMagneticDataReceived_updateAHRS;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error: " + ex.Message);
            }
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }

        /// <summary>
        /// xIMUserial CalInertialAndMagneticDataReceived event to update algorithm in AHRS mode.
        /// </summary>
        static void xIMUserial_CalInertialAndMagneticDataReceived_updateAHRS(object sender, x_IMU_API.CalInertialAndMagneticData e)
        {
            AHRS.Update(Deg2Rad(e.Gyroscope[0]), Deg2Rad(e.Gyroscope[1]), Deg2Rad(e.Gyroscope[2]), e.Accelerometer[0],
                e.Accelerometer[1], e.Accelerometer[2], e.Magnetometer[0], e.Magnetometer[1], e.Magnetometer[2]);

            var quaternion = new[] {AHRS.Quaternion[0], -AHRS.Quaternion[1], -AHRS.Quaternion[2], 0};
            var quaternionData = new QuaternionData(quaternion);
            
            var conjugate = quaternionData.ConvertToConjugate();
            var rotationMatrix = conjugate.ConvertToRotationMatrix();

            _form3DcuboidB.RotationMatrix = rotationMatrix;

            _stopWatch.Stop();
            _stopWatch.Reset();
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="degrees">
        /// Angular quantity in degrees.
        /// </param>
        /// <returns>
        /// Angular quantity in radians.
        /// </returns>
        private static float Deg2Rad(float degrees)
        {
            return (float)(Math.PI / 180) * degrees;
        }
    }

    public class QuadRotorImu
    {
        private readonly Stopwatch _stopWatch;

        public QuadRotorImu(Stopwatch stopWatch)
        {
            _stopWatch = stopWatch;
        }

        private void GetDataMeasurements(SerialPort serialPort)
        {
            if (!_stopWatch.IsRunning) _stopWatch.Start();
            //The below setting are for the Hello handshake
            var buffer = new[]
            {
                Convert.ToChar(16), Convert.ToChar(126), Convert.ToChar(4), Convert.ToChar(0),
                Convert.ToChar(serialPort.NewLine)
            };

            serialPort.Write(buffer, 0, buffer.Length);
        }

        private void CurrentPortOnDataReceived(object sender, SerialDataReceivedEventArgs serialDataReceivedEventArgs)
        {
            var port = sender as SerialPort;

            if (port == null) return;

            var returnMessage = port.ReadLine();

            if (string.IsNullOrEmpty(returnMessage))
            {
                GetDataMeasurements(port);
                return;
            }

            if (returnMessage.Contains("HELLO FROM ARDUINO"))
            {
                GetDataMeasurements(port);
                return;
            }

            var occurances = Regex.Matches(returnMessage, "@").Cast<Match>();

            if (occurances.Count() != 8)
            {
                GetDataMeasurements(port);
                return;
            }

            var result = new List<float>();
            while (result.Count != 9)
            {
                var end = returnMessage.IndexOf("@", StringComparison.Ordinal);
                var value = returnMessage.Substring(0, end != -1 ? end : returnMessage.Length);
                var shift = end != -1 ? end + 1 : 0;
                returnMessage = returnMessage.Substring(shift, returnMessage.Length - shift);
                result.Add(Convert.ToSingle(value));
            }

            var gyroscope = new[] {result[0], result[1], result[2]};
            var accelerometer = new[] {result[3], result[4], result[5]};
            var magnitometer = new[] {result[6], result[7], result[8]};

            CalInertialAndMagneticDataReceived?.Invoke(this,
                new CalInertialAndMagneticData
                {
                    Accelerometer = accelerometer,
                    Gyroscope = gyroscope,
                    Magnetometer = magnitometer
                });
            
            GetDataMeasurements(port);
        }

        public void Open()
        {
            SetComPort(); 
        }

        public event xIMUserial.onCalInertialAndMagneticDataReceived CalInertialAndMagneticDataReceived;

        private SerialPort _currentPort;

        private void SetComPort()
        {
            foreach (var port in SerialPort.GetPortNames())
            {
                var currentPort = new SerialPort(port, 115200);
                DetectArduino(currentPort);
            }
        }

        private void SerialPortOnHandShake(object sender, SerialDataReceivedEventArgs serialDataReceivedEventArgs)
        {
            var serialPort = sender as SerialPort;
            if (serialPort == null) return;
            
            var count = serialPort.BytesToRead;
            if (count == 0)
            {
                serialPort.Close();
                serialPort.DataReceived -= SerialPortOnHandShake;
                return;
            }

            var returnMessage = serialPort.ReadTo(serialPort.NewLine);

            if (returnMessage.Contains("HELLO FROM ARDUINO"))
            {
                _currentPort = serialPort;
                serialPort.DataReceived -= SerialPortOnHandShake;
                _currentPort.DataReceived += CurrentPortOnDataReceived;
                GetDataMeasurements(_currentPort);
                return;
            }

            DetectArduino(serialPort);
        }

        private void DetectArduino(SerialPort serialPort)
        {
            var buffer = new[]
            {
                Convert.ToChar(16), Convert.ToChar(127), Convert.ToChar(0), Convert.ToChar(0),
                Convert.ToChar(serialPort.NewLine)
            };

            if (!serialPort.IsOpen)
            {
                serialPort.Open();
                serialPort.DataReceived -= SerialPortOnHandShake;
                serialPort.DataReceived += SerialPortOnHandShake;
            }

            serialPort.Write(buffer, 0, buffer.Length);
        }
    }
}