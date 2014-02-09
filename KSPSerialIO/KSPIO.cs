using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.Win32;
using System.Runtime.InteropServices;

using OpenNETCF.IO.Ports;
using UnityEngine;
using KSP.IO;

namespace KSPSerialIO
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VesselData
    {
        public byte id;             //1
        public float AP;            //2
        public float PE;            //3
        public float SemiMajorAxis; //4
        public float SemiMinorAxis; //5
        public float VVI;           //6
        public float e;             //7
        public float inc;           //8
        public float G;             //9
        public int TAp;             //10
        public int TPe;             //11
        public float TrueAnomaly;   //12
        public float Density;       //13
        public int period;          //14
        public float RAlt;          //15
        public float Fuelp;         //16
        public float Vsurf;         //17
        public float Lat;           //18
        public float Lon;           //19
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct HandShakePacket
    {
        public byte id;
        public byte M1;
        public byte M2;
        public byte M3;
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class SettingsNStuff : MonoBehaviour
    {
        public static PluginConfiguration cfg = PluginConfiguration.CreateForType<SettingsNStuff>();
        public static string DefaultPort;
        public static double refreshrate;
        public static int HandshakeDelay;
        public static int BaudRate;

        void Awake()
        {
            //cfg["refresh"] = 0.08;
            //cfg["DefaultPort"] = "COM1";
            //cfg["HandshakeDelay"] = 2500;
            print("KSPSerialIO: Loading settings...");

            cfg.load();
            DefaultPort = cfg.GetValue<string>("DefaultPort");
            print("KSPSerialIO: Default Port = " + DefaultPort);

            refreshrate = cfg.GetValue<double>("refresh");
            print("KSPSerialIO: Refreshrate = " + refreshrate.ToString());

            BaudRate = cfg.GetValue<int>("BaudRate");
            print("KSPSerialIO: BaudRate = " + BaudRate.ToString());

            HandshakeDelay = cfg.GetValue<int>("HandshakeDelay");
            print("KSPSerialIO: Handshake Delay = " + HandshakeDelay.ToString());
        }
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class KSPSerialPort : MonoBehaviour
    {
        public static SerialPort Port;
        public static string PortNumber;
        public static VesselData VData;
        public static Boolean DisplayFound = false;
        private static HandShakePacket HPacket;
        private string COMRx;

        public static void sendPacket(object anything)
        {
            byte[] Payload = StructureToByteArray(anything);
            byte header1 = 0xBE;
            byte header2 = 0xEF;
            byte size = (byte)Payload.Length;
            byte checksum = size;

            byte[] Packet = new byte[size + 4];

            //Packet = [header][size][payload][checksum];
            //Header = [Header1=0xBE][Header2=0xEF]
            //size = [payload.length (0-255)]

            for (int i = 0; i < size; i++)
            {
                checksum ^= Payload[i];
            }

            Payload.CopyTo(Packet, 3);
            Packet[0] = header1;
            Packet[1] = header2;
            Packet[2] = size;
            Packet[Packet.Length - 1] = checksum;

            Port.Write(Packet, 0, Packet.Length);
        }

        //this thing is copied from the intarwebs, converts struct to byte array
        private static byte[] StructureToByteArray(object obj)
        {
            int len = Marshal.SizeOf(obj);
            byte[] arr = new byte[len];
            IntPtr ptr = Marshal.AllocHGlobal(len);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, len);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        void initializeDataPackets()
        {
            VData = new VesselData();
            VData.id = 1;

            HPacket = new HandShakePacket();
            HPacket.id = 0x00;
            HPacket.M1 = 0x4B;
            HPacket.M2 = 0x53;
            HPacket.M3 = 0x50;
        }

        void Awake()
        {
            print("KSPSerialIO: Version 0.12");
            print("KSPSerialIO: Getting serial ports...");
            initializeDataPackets();

            try
            {
                //Use registry hack to get a list of serial ports until we get system.io.ports
                RegistryKey SerialCOMSKey = Registry.LocalMachine.OpenSubKey(@"HARDWARE\DEVICEMAP\SERIALCOMM\");

                if (SerialCOMSKey == null)
                {
                    print("KSPSerialIO: Dude do you even win32 serial port??");
                }
                else
                {
                    String[] realports = SerialCOMSKey.GetValueNames();  // get list of all serial devices
                    String[] names = new string[realports.Length + 1];   // make a new list with 1 extra, we put the default port first
                    realports.CopyTo(names, 1);

                    print("KSPSerialIO: Found " + names.Length.ToString() + " serial ports");

                    //look through all found ports for our display
                    int j = 0;

                    foreach (string PortName in names)
                    {
                        if (j == 0)  // try default port first
                        {
                            PortNumber = SettingsNStuff.DefaultPort;
                            print("KSPSerialIO: trying default port " + PortNumber);
                        }
                        else
                        {
                            PortNumber = (string)SerialCOMSKey.GetValue(PortName);
                            print("KSPSerialIO: trying port " + PortName + " - " + PortNumber);
                        }

                        j++;

                        Port = new SerialPort(PortNumber, SettingsNStuff.BaudRate, Parity.None, 8, StopBits.One);
                        Port.ReceivedBytesThreshold = 3;

                        //print("KSPSerialIO: receive threshold " + Port.ReceivedBytesThreshold.ToString());

                        Port.ReceivedEvent += Port_ReceivedEvent;

                        if (!Port.IsOpen)
                        {
                            try
                            {
                                Port.Open();
                            }
                            catch (Exception e)
                            {
                                print("Error opening serial port " + Port.PortName + ": " + e.Message);
                            }

                            //secret handshake
                            if (Port.IsOpen)
                            {
                                Thread.Sleep(SettingsNStuff.HandshakeDelay);
                                sendPacket(HPacket);

                                //wait for reply
                                int k = 0;
                                while (Port.BytesToRead == 0 && k < 5 && !DisplayFound)
                                {
                                    Thread.Sleep(200);
                                    k++;
                                }

                                Port.Close();
                                if (DisplayFound)
                                {
                                    print("KSPSerialIO: found KSP Display at " + Port.PortName);
                                    break;
                                }
                                else
                                {
                                    print("KSPSerialIO: KSP Display not found");
                                }
                            }
                        }
                        else
                        {
                            print("KSPSerialIO: " + PortNumber + "is already being used.");
                        }
                    }
                }

                //ports = SerialPort.GetPortNames();
            }
            catch (Exception e)
            {
                print(e.Message);
            }
        }

        private string readline()
        {
            string result = null;
            char c;
            int j = 0;

            c = (char)Port.ReadByte();
            while (c != '\n' && j < 255)
            {
                result += c;
                c = (char)Port.ReadByte();
                j++;
            }
            return result;
        }

        private void Port_ReceivedEvent(object sender, SerialReceivedEventArgs e)
        {
            //COMRx = "just fucking work already!!!";
            while (Port.BytesToRead > 0)
            {
                try
                {
                    COMRx = readline();
                    ProcessCOMRx();
                }
                catch
                {
                    ReadLineError();
                }
            }
        }

        private void ReadLineError()
        {
            print("Read Line Error");
        }

        private void ProcessCOMRx()
        {
            if (!string.IsNullOrEmpty(COMRx))
            {
                try
                {
                    string[] parsed = COMRx.Split(';');

                    if (parsed.Length > 1)
                    {
                        if (parsed[0] == "KSP")
                        {
                            short messageID = (short)double.Parse(parsed[1]);
                            if (messageID == 0)
                            {
                                DisplayFound = true;
                            }
                        }
                    }
                    COMRx = "";
                }
                catch
                {
                    print("Parse Error: " + COMRx + '\n');
                }
            }
        }

        void OnDestroy()
        {
            if (KSPSerialPort.Port.IsOpen)
            {
                KSPSerialPort.Port.Close();
                print("KSPSerialIO: Port closed");
            }
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPSerialIO : MonoBehaviour
    {
        private double lastUpdate = 0.0f;
        public double refreshrate = 1.0f;

        private ScreenMessageStyle KSPIOScreenStyle = ScreenMessageStyle.LOWER_CENTER;
        Vessel ActiveVessel;

        void Awake()
        {
            ScreenMessages.PostScreenMessage("IO awake", 10f, KSPIOScreenStyle);
            refreshrate = SettingsNStuff.refreshrate;
        }

        void Start()
        {
            if (KSPSerialPort.DisplayFound)
            {
                if (!KSPSerialPort.Port.IsOpen)
                {
                    ScreenMessages.PostScreenMessage("Starting serial port " + KSPSerialPort.Port.PortName, 10f, KSPIOScreenStyle);

                    try
                    {
                        KSPSerialPort.Port.Open();
                    }
                    catch (Exception e)
                    {
                        ScreenMessages.PostScreenMessage("Error opening serial port " + KSPSerialPort.Port.PortName, 10f, KSPIOScreenStyle);
                        ScreenMessages.PostScreenMessage(e.Message, 10f, KSPIOScreenStyle);
                    }
                }
                else
                {
                    ScreenMessages.PostScreenMessage("Using serial port " + KSPSerialPort.Port.PortName, 10f, KSPIOScreenStyle);
                }
            }
            else
            {
                ScreenMessages.PostScreenMessage("No display found", 10f, KSPIOScreenStyle);
            }
        }

        void Update()
        {
            if ((Time.time - lastUpdate) > refreshrate && KSPSerialPort.Port.IsOpen)
            {
                ActiveVessel = FlightGlobals.ActiveVessel;

                if (ActiveVessel != null)
                {
                    lastUpdate = Time.time;

                    KSPSerialPort.VData.AP = (float)ActiveVessel.orbit.ApA;
                    KSPSerialPort.VData.PE = (float)ActiveVessel.orbit.PeA;
                    KSPSerialPort.VData.SemiMajorAxis = (float)ActiveVessel.orbit.semiMajorAxis;
                    KSPSerialPort.VData.SemiMinorAxis = (float)ActiveVessel.orbit.semiMinorAxis;
                    KSPSerialPort.VData.e = (float)ActiveVessel.orbit.eccentricity;
                    KSPSerialPort.VData.inc = (float)ActiveVessel.orbit.inclination;
                    KSPSerialPort.VData.VVI = (float)ActiveVessel.verticalSpeed;
                    KSPSerialPort.VData.G = (float)ActiveVessel.geeForce;
                    KSPSerialPort.VData.TAp = (int)Math.Round(ActiveVessel.orbit.timeToAp);
                    KSPSerialPort.VData.TPe = (int)Math.Round(ActiveVessel.orbit.timeToPe);
                    KSPSerialPort.VData.Density = (float)ActiveVessel.atmDensity;
                    KSPSerialPort.VData.TrueAnomaly = (float)ActiveVessel.orbit.trueAnomaly;
                    KSPSerialPort.VData.period = (int)Math.Round(ActiveVessel.orbit.period);


                    double ASL = ActiveVessel.mainBody.GetAltitude(ActiveVessel.CoM);
                    double AGL = (ASL - ActiveVessel.terrainAltitude);

                    if (AGL < ASL)
                        KSPSerialPort.VData.RAlt = (float)AGL;
                    else
                        KSPSerialPort.VData.RAlt = (float)ASL;

                    KSPSerialPort.VData.Fuelp = GetResourcePercent(ActiveVessel, "LiquidFuel");

                    KSPSerialPort.VData.Vsurf = (float)ActiveVessel.srfSpeed;
                    KSPSerialPort.VData.Lat = (float)ActiveVessel.latitude;
                    KSPSerialPort.VData.Lon = (float)ActiveVessel.longitude;

                    //ScreenMessages.PostScreenMessage(KSPSerialPort.VData.Fuelp.ToString());
                    //ScreenMessages.PostScreenMessage(KSPSerialPort.VData.RAlt.ToString());
                    //KSPSerialPort.Port.WriteLine("Success!");
                    KSPSerialPort.sendPacket(KSPSerialPort.VData);
                }
            }
        }

        private float GetResourcePercent(Vessel V, string resourceName)
        {
            double Fuel = 0;
            double FuelMax = 0;

            foreach (Part p in ActiveVessel.parts)
            {
                foreach (PartResource pr in p.Resources)
                {
                    if (pr.resourceName.Equals(resourceName))
                    {
                        Fuel += pr.amount;
                        FuelMax += pr.maxAmount;
                        break;
                    }
                }
            }

            if (FuelMax == 0)
                return 0.0f;
            else
                return (float)(Fuel / FuelMax * 100.0);
        }

        void FixedUpdate()
        {
        }

        void OnDestroy()
        {
            if (KSPSerialPort.Port.IsOpen)
            {
                KSPSerialPort.Port.Close();
                ScreenMessages.PostScreenMessage("Port closed", 10f, KSPIOScreenStyle);
            }
        }
    }
}
