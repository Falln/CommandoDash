﻿using FRC.CameraServer;
using FRC.NetworkTables;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using MjpegProcessor;
using System.Diagnostics;

namespace CommandoDash
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// Dashboard px: 1920*230
    public partial class MainWindow : Window
    {
        NetworkTableInstance ntInst;
        NetworkTable FMSInfoNT;
        NetworkTable CommandoDashNT;
        MjpegDecoder CDDCameraStream;

        string cameraURI = "";
        string robotIP = "10.58.59.2";
        string limelightIP = "10.58.89.11";
        string photonIP = "10.58.89.12";
        
        public MainWindow()
        {
            InitializeComponent();

            //NT Startup
            ntInst = NetworkTableInstance.Default;
            ntInst.StartClientTeam(5889);
            ntInst.StartDSClient();

            //Camera Startup
            CDDCameraStream = new MjpegDecoder();
            startCamera();

            //NT Tables
            FMSInfoNT = ntInst.GetTable("FMSInfo");
            CommandoDashNT = ntInst.GetTable("CommandoDash");

            //Setup Default States
            defaultStates();

            //NT Listeners
            //Update RobotConnectionStatus on connection to the NT
            ntInst.AddConnectionListener(
                (in ConnectionNotification _) => Dispatcher.Invoke(
                       new Action(() => updateRobotConnectionStatus())),
                true);

            //Robot Mode update listener
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("AllianceAndModeData").GetEntry("robotMode"), new Action(() => updateRobotMode()));

            //Update the what the RIO thinks the Auto selected is
            AddSimpleEntryListener(CommandoDashNT.GetEntry("rioAutoSelection"), new Action(() => rioAutoSelection_Update()));

            //Update Dash based on current Alliance Color
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("AllianceAndModeData").GetEntry("alliance"), new Action(() => alliance_Update()));

            //Update the status blocks with the relevant data
            //Is Hounding
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("isHounding"), new Action(() =>
                HoundHoundingTargetSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("isHounding").GetBoolean(false)));

            //Is AutoAiming
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("isAutoAiming"), new Action(() =>
                LimeTrackingTargetSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("isAutoAiming").GetBoolean(false)));

            //Seeing Hub
            ntInst.GetTable("limelight").GetEntry("tv").SetDouble(0); //TODO Delete all these testing sets
            AddSimpleEntryListener(ntInst.GetTable("limelight").GetEntry("tv"), new Action(() => 
                LimeSeesTargetSB.IsActive = ntInst.GetTable("limelight").GetEntry("tv").GetDouble(0) == 1 ? true:false));

            //Seeing a Cargo
            AddSimpleEntryListener(ntInst.GetTable("photonvision").GetSubTable("CargoHound").GetEntry("hasTargets"), new Action(() =>
                HoundSeesTargetSB.IsActive = ntInst.GetTable("photonvision").GetSubTable("CargoHound").GetEntry("hasTargets").GetBoolean(false)));

            //Intake Solenoid
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("intakeSolenoidState"), new Action(() =>
                IntakeSolSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("intakeSolenoidState").GetBoolean(false)));

            //Mid Climb Solenoid
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("midSolenoidState"), new Action(() =>
                MidSolSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("midSolenoidState").GetBoolean(false)));

            //Traversal Climb Solenoid
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("traversalSolenoidState"), new Action(() =>
                TraversalSolSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("traversalSolenoidState").GetBoolean(false)));

            //Update Shooter RPM
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("shooterRPM"), new Action(() =>
                ShooterRPMBox.Text = CommandoDashNT.GetSubTable("SensorData").GetEntry("shooterRPM").GetDouble(0).ToString()));

            //Update Manual CycleSpeed
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("manualCycleSpeed"), new Action(() =>
                ManualCycleBox.Text = CommandoDashNT.GetSubTable("SensorData").GetEntry("manualCycleSpeed").GetDouble(0).ToString()));

            //Update ReadyToFire
            AddSimpleEntryListener(CommandoDashNT.GetEntry("readyToFire"), new Action(() =>
                ReadyToFireSB.IsActive = CommandoDashNT.GetEntry("readyToFire").GetBoolean(false)));

            //Update Current Usage
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("PowerUsage").GetEntry("batteryVoltage"), new Action(() => updatePowerUsage()));

        }

        public void defaultStates()
        {
            //Initialize any default states of the WPF Controls
            IntakeCamRadio.IsChecked = true;
            IdealRadio.IsChecked = true;
            cameraURI = CameraURIInput.Text;
            robotIP = robotIPInput.Text;
            cameraStream.Source = new BitmapImage(new Uri("Images/Logo_Square_WhiteBackground_CommandoRobotics.png", UriKind.Relative));
        }

        public void AddSimpleEntryListener(NetworkTableEntry entry, Action function)
        {
            entry.AddListener((in RefEntryNotification _) =>
                Dispatcher.Invoke(function),
                NotifyFlags.New | NotifyFlags.Update);
        }

        public void startCamera()
        {
            //Check if there is a stream already active
            if (CDDCameraStream != null)
            {
                CDDCameraStream.StopStream();
            }

            //Start the new camera stream decoder
            CDDCameraStream = new MjpegDecoder();
            CDDCameraStream.FrameReady += cameraStream_FrameReady;
            CDDCameraStream.Error += cameraStream_Error;
            try
            {
                CDDCameraStream.ParseStream(new Uri(cameraURI));
            } catch (Exception)
            {
                CameraURIInput.Text = "Could not find URI";
                CDDCameraStream.StopStream();
                cameraStream.Source = new BitmapImage(new Uri("Images/Logo_Square_WhiteBackground_CommandoRobotics.png", UriKind.Relative));
            }
        }

        public void cameraStream_FrameReady(object sender, FrameReadyEventArgs e)
        {
            cameraStream.Source = e.BitmapImage;
        }
        public void cameraStream_Error(object sender, ErrorEventArgs e)
        {
            cameraStream.Source = new BitmapImage(new Uri("Images/Logo_Square_WhiteBackground_CommandoRobotics.png", UriKind.Relative));
        }

        private void updateRobotMode()
        {
            double robotMode = CommandoDashNT.GetSubTable("AllianceAndModeData").GetEntry("robotMode").GetDouble(0);
            if (!ntInst.IsConnected())
            {
                robotModeBlock.Text = "DISCONNECTED";
                robotModeBlock.Background = Brushes.Transparent;
            }
            else if (robotMode == 0)
            {
                robotModeBlock.Text = "DISCONNECTED";
                robotModeBlock.Background = Brushes.Transparent;
            }
            else if (robotMode == 33)
            {
                robotModeBlock.Text = "TeleOp Enabled";
                robotModeBlock.Background = Brushes.Blue;
            }
            else if (robotMode == 35)
            {
                robotModeBlock.Text = "Auto Enabled";
                robotModeBlock.Background = Brushes.Purple;
            }
            else if (robotMode == 37)
            {
                robotModeBlock.Text = "Test Enabled";
                robotModeBlock.Background = Brushes.Yellow;
            }
            else
            {
                robotModeBlock.Text = "Disabled";
                robotModeBlock.Background = Brushes.Gray;
            }
        }

        private void updateRobotConnectionStatus()
        {
            if (ntInst.IsConnected())
            {
                robotStatus.Background = Brushes.Green;
                robotIPInput.Text = ntInst.GetConnections()[0].RemoteIp;
                robotIP = robotIPInput.Text;
                updateRobotMode();
            }
            else
            {
                robotStatus.Background = Brushes.Red;
                updateRobotMode();
            }
        }

        private void rioAutoSelection_Update()
        {
            RioAutoBox.Text = CommandoDashNT.GetEntry("rioAutoSelection").GetString("");
        }

        private void alliance_Update()
        {
            if (CommandoDashNT.GetSubTable("AllianceAndModeData").GetEntry("alliance").GetDouble(0) == 1)
            {
                HoundHoundingTargetSB.ActiveBrush = (Brush) this.Resources["HoundRedHounding"];
                HoundHoundingTargetSB.InactiveBrush = (Brush) this.Resources["HoundRedDefault"];
                HoundHoundingTargetSB.BorderBrush = (Brush)this.Resources["HoundRedBorder"];
                HoundSeesTargetSB.ActiveBrush = (Brush) this.Resources["HoundRedHounding"];
                HoundSeesTargetSB.InactiveBrush = (Brush) this.Resources["HoundRedDefault"];
                HoundSeesTargetSB.BorderBrush = (Brush)this.Resources["HoundRedBorder"];
            } else
            {
                HoundHoundingTargetSB.ActiveBrush = (Brush)this.Resources["HoundBlueHounding"];
                HoundHoundingTargetSB.InactiveBrush = (Brush)this.Resources["HoundBlueDefault"];
                HoundHoundingTargetSB.BorderBrush = (Brush)this.Resources["HoundBlueBorder"];
                HoundSeesTargetSB.ActiveBrush = (Brush)this.Resources["HoundBlueHounding"];
                HoundSeesTargetSB.InactiveBrush = (Brush)this.Resources["HoundBlueDefault"];
                HoundSeesTargetSB.BorderBrush = (Brush)this.Resources["HoundBlueBorder"];
            }
            
        }

        private void updatePowerUsage()
        {
            NetworkTable powerTable = CommandoDashNT.GetSubTable("PowerUsage");
            //Update all power ProgressBoxes and TextBoxes
            //Drive
            double driveFLCurrent = powerTable.GetEntry("driveFLCurrent").GetDouble(0);
            double driveFRCurrent = powerTable.GetEntry("driveFRCurrent").GetDouble(0);
            double driveRLCurrent = powerTable.GetEntry("driveRLCurrent").GetDouble(0);
            double driveRRCurrent = powerTable.GetEntry("driveRRCurrent").GetDouble(0);
            double averageDriveCurrent = (driveFLCurrent + driveFRCurrent + driveRLCurrent + driveRRCurrent) / 4;
            DrivePowerProgress.Value = DrivePowerProgress.Maximum - averageDriveCurrent;
            DrivePowerText.Text = averageDriveCurrent.ToString();

            //Shooter
            double shooterLCurrent = powerTable.GetEntry("shooterLCurrent").GetDouble(0);
            double shooterRCurrent = powerTable.GetEntry("shooterRCurrent").GetDouble(0);
            ShooterPowerProgress.Value = ShooterPowerProgress.Maximum - ((shooterLCurrent + shooterRCurrent)/2);
            ShooterPowerText.Text = ((shooterLCurrent + shooterRCurrent)/2).ToString();

            //Intake
            double intakeCurrent = powerTable.GetEntry("intakeCurrent").GetDouble(0);
            IntakePowerProgress.Value = IntakePowerProgress.Maximum - intakeCurrent;
            IntakePowerText.Text = intakeCurrent.ToString();

            //Index
            double rampCurrent = powerTable.GetEntry("rampCurrent").GetDouble(0);
            double verticalCurrent = powerTable.GetEntry("verticalCurrent").GetDouble(0);
            IndexPowerProgress.Value = IntakePowerProgress.Maximum - ((verticalCurrent + rampCurrent)/2);
            IndexPowerText.Text = ((verticalCurrent + rampCurrent) / 2).ToString();

            //Transfer
            double transferLCurrent = powerTable.GetEntry("transferLCurrent").GetDouble(0);
            double transferRCurrent = powerTable.GetEntry("transferRCurrent").GetDouble(0);
            TransferPowerProgress.Value = TransferPowerProgress.Maximum - ((transferRCurrent + transferLCurrent) / 2);
            TransferPowerText.Text = ((transferRCurrent + transferLCurrent) / 2).ToString();

            //VRM
            double vrmCurrent = powerTable.GetEntry("VRMCurrent").GetDouble(0);
            VRMPowerProgress.Value = VRMPowerProgress.Maximum - vrmCurrent;
            VRMPowerText.Text = vrmCurrent.ToString();

            //Overall Current and Battery Voltage
            TotalCurrentSB.Text = powerTable.GetEntry("totalCurrent").GetDouble(0).ToString();
            BatteryVolatgeSB.Text = powerTable.GetEntry("batteryVoltage").GetDouble(0).ToString();
        }


        //Clicks
        private void closeBtn_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        private void minimizeBtn_Click(object sender, RoutedEventArgs e)
        {
            this.WindowState = WindowState.Minimized;
        }

        private void makeResizeBtn_Click(object sender, RoutedEventArgs e)
        {
            if (this.WindowStyle.Equals(WindowStyle.SingleBorderWindow))
            {
                this.WindowStyle = WindowStyle.None;
                this.Height = 850;
                this.Width = 1920;
                this.Left = 0;
                this.Top = 0;
                this.ResizeMode = ResizeMode.NoResize;
            }
            else
            {
                this.WindowStyle = WindowStyle.SingleBorderWindow;
                this.ResizeMode = ResizeMode.CanResize;
            }
        }

        private void refreshBtn_Click(object sender, RoutedEventArgs e)
        {
            var currentExecutablePath = Process.GetCurrentProcess().MainModule.FileName;
            Process.Start(currentExecutablePath);
            Application.Current.Shutdown();
        }

        private void robotIPInput_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                ntInst.StopClient();
                ntInst.StopServer();
                ntInst.StopDSClient();
                ntInst.SetServer(robotIPInput.Text);
                ntInst.StartClient();
                FMSInfoNT = ntInst.GetTable("FMSInfo");
                updateRobotMode();
            }
        }

        private void IntakeCamRadio_Checked(object sender, RoutedEventArgs e)
        {
            cameraURI = "http://" + photonIP + ":1181/stream.mjpg";
            CameraURIInput.Text = cameraURI;
            startCamera();
        }

        private void ShooterCamRadio_Checked(object sender, RoutedEventArgs e)
        {
            ntInst.GetTable("limelight").GetEntry("camMode").SetDouble(1);
            cameraURI = "http://" + limelightIP + ":5800/stream.mjpg";
            CameraURIInput.Text = cameraURI;
            startCamera();
        }

        private void ShooterCamRadio_Unchecked(object sender, RoutedEventArgs e)
        {
            ntInst.GetTable("limelight").GetEntry("camMode").SetDouble(0);
        }

        private void OverlayCamRadio_Checked(object sender, RoutedEventArgs e)
        {
            cameraURI = "No known IP";
            CameraURIInput.Text = cameraURI;
            startCamera();
        }

        private void IntakeThreshRadio_Checked(object sender, RoutedEventArgs e)
        {
            cameraURI = "http://" + photonIP + ":1182/stream.mjpg";
            CameraURIInput.Text = cameraURI;
            startCamera();
        }

        private void ShooterThreshRadio_Checked(object sender, RoutedEventArgs e)
        {
            ntInst.GetTable("limelight").GetEntry("camMode").SetDouble(0);
            cameraURI = "http://" + limelightIP + ":5800/stream.mjpg";
            CameraURIInput.Text = cameraURI;
            startCamera();
        }

        private void RestartCameraConnection_Click(object sender, RoutedEventArgs e)
        {
            startCamera();
            CameraURIInput.Text = cameraURI;
        }

        private void CameraURIInput_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                IntakeCamRadio.IsChecked = false;
                ShooterCamRadio.IsChecked = false;
                OverlayCamRadio.IsChecked = false;
                IntakeThreshRadio.IsChecked = false;
                ShooterThreshRadio.IsChecked = false;
                cameraURI = CameraURIInput.Text;
            }

        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("testData");
            entry.SetBoolean(!entry.GetBoolean(false));
            IntakeSolSB.IsActive = !IntakeSolSB.IsActive;
            LimeTrackingTargetSB.IsActive = !LimeTrackingTargetSB.IsActive;
            HoundHoundingTargetSB.IsActive = !HoundHoundingTargetSB.IsActive;
        }

        private void TaxiRadio_Checked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("Taxi");
        }

        private void SpareRadio_Checked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("Spare");
        }

        private void IdealRadio_Checked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("IdealAuto");
        }

        private void SecondAutoRadio_Checked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("SecondAuto");
        }

        private void FullSendRadio_Checked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("FullSend");
        }
    }
}
