using FRC.CameraServer;
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
            } catch (Exception _)
            {
                CameraURIInput.Text = "Could not find URI";
                CDDCameraStream.StopStream();
                cameraStream.Source = new BitmapImage(new Uri("Images/Logo_Square_WhiteBackground_CommandoRobotics.png", UriKind.Relative));
            }
        }

        public class StatusTextBlock : TextBlock
        {
            public static readonly DependencyProperty IsActiveProperty = DependencyProperty.Register(
                "IsActive", typeof(bool),
                typeof(TextBlock)
            );

            public bool IsActive
            {
                get => (bool)GetValue(IsActiveProperty);
                set => SetValue(IsActiveProperty, value);
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
