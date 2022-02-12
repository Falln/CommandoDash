using FRC.NetworkTables;
using NetworkTables.Tables;
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

namespace CommandoDash
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    //Dashboard px: 1920*230
    public partial class MainWindow : Window
    {
        NetworkTableInstance ntInst;
        NetworkTable FMSInfoNT;
        public MainWindow()
        {
            InitializeComponent();
            ntInst = NetworkTableInstance.Default;
            ntInst.StartClientTeam(5889);
            ntInst.StartDSClient();
            FMSInfoNT = ntInst.GetTable("FMSInfo");
            ntInst.AddConnectionListener(
                (in ConnectionNotification _) => Dispatcher.Invoke(
                       new Action(() => updateRobotConnectionStatus())),
                true);

            AddSimpleEntryListener(FMSInfoNT, "FMSControlData", new Action(() => updateRobotMode()));
        }

        public void AddSimpleEntryListener(NetworkTable table, String key, Action function)
        {
            table.GetEntry(key).AddListener((in RefEntryNotification _) =>
                Dispatcher.Invoke(function),
                NotifyFlags.New | NotifyFlags.Update);
        }

        private void updateRobotMode()
        {
            double robotMode = FMSInfoNT.GetEntry("FMSControlData").GetDouble(0);
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
                updateRobotMode();
            }
            else
            {
                robotStatus.Background = Brushes.Red;
                updateRobotMode();
            }
        }

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
    }
}
