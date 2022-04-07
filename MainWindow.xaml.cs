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
        NetworkTable PIDTuningNT;
        MjpegDecoder CDDCameraStream;

        string cameraURI = "";
        string robotIP = "10.58.59.2";
        string limelightIP = "10.58.89.11";
        string photonIP = "10.58.89.12";

        double robotXFieldOffset = 23;
        double robotYFieldOffset = 18;

        double timesPowerUpdated = 0;
        double targetNumberTimesPowerUpdated = 1;
        
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
            PIDTuningNT = CommandoDashNT.GetSubTable("PIDTuning");

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
            AddSimpleEntryListener(ntInst.GetTable("photonvision").GetSubTable("CargoHound").GetEntry("hasTarget"), new Action(() =>
                HoundSeesTargetSB.IsActive = ntInst.GetTable("photonvision").GetSubTable("CargoHound").GetEntry("hasTarget").GetBoolean(false)));

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
                ShooterRPMBox.Text = Math.Round(CommandoDashNT.GetSubTable("SensorData").GetEntry("shooterRPM").GetDouble(0), 0).ToString()));

            //Update Manual CycleSpeed
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("manualCycleSpeed"), new Action(() =>
                ManualCycleBox.Text = CommandoDashNT.GetSubTable("SensorData").GetEntry("manualCycleSpeed").GetDouble(0).ToString()));

            //Update VectorMap
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("targetRPM"), new Action(() => updateVectorMapBox()));

            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("vectorMapRange"), new Action(() => updateVectorMapBox()));

            //Update ReadyToFire
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("isRobotAimed"), new Action(() =>
                ReadyToFireSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("isRobotAimed").GetBoolean(false) && CommandoDashNT.GetSubTable("SensorData").GetEntry("isAtTargetVelocity").GetBoolean(false)));

            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("isAtTargetVelocity"), new Action(() =>
                ReadyToFireSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("isRobotAimed").GetBoolean(false) && CommandoDashNT.GetSubTable("SensorData").GetEntry("isAtTargetVelocity").GetBoolean(false)));

            //Update Current Usage
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("PowerUsage").GetEntry("batteryVoltage"), new Action(() => updatePowerUsage()));

            //Update the current gyro angle
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("gyroAngle"), new Action(() => updateGyroInfo()));

            //Update if we're in centric mode
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("isCentric"), new Action(() =>
                IsCentricSB.IsActive = CommandoDashNT.GetSubTable("SensorData").GetEntry("isCentric").GetBoolean(true)));

            //Update Index Spot 1
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("IndexStatus").GetEntry("verticalSensor"), new Action(() => updateIndexSpot1()));

            //Update Index Spot 2
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("IndexStatus").GetEntry("rampSensor"), new Action(() => updateIndexSpot2()));

            //Update Intake Spot 
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("IndexStatus").GetEntry("entranceSensor"), new Action(() => updateIntakeSpot()));

            //Update Intake Transfer Spot
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("IndexStatus").GetEntry("entranceToRampBlock"), new Action(() =>
                IntakeCargoTransfer.Visibility = CommandoDashNT.GetSubTable("IndexStatus").GetEntry("entranceToRampBlock").GetBoolean(false) ? Visibility.Visible : Visibility.Hidden));

            //Update Vertical Transfer Spot
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("IndexStatus").GetEntry("rampToVerticalBlock"), new Action(() => 
                VerticalCargoTransfer.Visibility = CommandoDashNT.GetSubTable("IndexStatus").GetEntry("rampToVerticalBlock").GetBoolean(false) ? Visibility.Visible : Visibility.Hidden));

            //AutoLayouts

            //Update Robot Field2d position
            AddSimpleEntryListener(ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("Robot"), new Action(() => updateRobotPosition()));

            //Update Cargo Field2d position
            AddSimpleEntryListener(ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("SeenCargo"), new Action(() => updateCargoPosition()));

            //PIDTuning Updates
            //Update shooter setpoint
            AddSimpleEntryListener(PIDTuningNT.GetEntry("shooterSetpoint"), new Action(() =>
                shooterSetpointBox.Text = Math.Round(PIDTuningNT.GetEntry("shooterSetpoint").GetDouble(0), 1).ToString()));

            //Hound X Y and R setpoint updates
            AddSimpleEntryListener(PIDTuningNT.GetEntry("houndXSetpoint"), new Action(() =>
                houndXSetpointBox.Text = Math.Round(PIDTuningNT.GetEntry("houndXSetpoint").GetDouble(0), 1).ToString()));

            AddSimpleEntryListener(PIDTuningNT.GetEntry("houndYSetpoint"), new Action(() =>
                houndYSetpointBox.Text = Math.Round(PIDTuningNT.GetEntry("houndYSetpoint").GetDouble(0), 1).ToString()));

            AddSimpleEntryListener(PIDTuningNT.GetEntry("houndRSetpoint"), new Action(() =>
                houndRSetpointBox.Text = Math.Round(PIDTuningNT.GetEntry("houndRSetpoint").GetDouble(0), 1).ToString()));

            //Update where the tilt arm should be
            AddSimpleEntryListener(CommandoDashNT.GetSubTable("SensorData").GetEntry("tiltAngle"), new Action(() =>
                ClimbArmAngle.Angle = -CommandoDashNT.GetSubTable("SensorData").GetEntry("tiltAngle").GetDouble(0)-5));

        }

        public void defaultStates()
        {
            //Initialize any default states of the WPF Controls
            IntakeCamRadio.IsChecked = true;
            //Auto Clicks
            IdealRadio.IsChecked = true;
            IdealRadioAL.IsChecked = true;
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("IdealAuto");
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
                shooterPBox.Text = PIDTuningNT.GetEntry("shooterP").GetDouble(0).ToString();
                shooterDBox.Text = PIDTuningNT.GetEntry("shooterD").GetDouble(0).ToString();
                shooterKsBox.Text = PIDTuningNT.GetEntry("shooterKs").GetDouble(0).ToString();
                shooterKvBox.Text = PIDTuningNT.GetEntry("shooterKv").GetDouble(0).ToString();
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
            RioAutoBoxAL.Text = CommandoDashNT.GetEntry("rioAutoSelection").GetString("");
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

                FieldCargo.ImageSource = new BitmapImage(new Uri("Images/Red_Cargo.png", UriKind.Relative));
                FieldImageScale.ScaleX = -1;
                FieldImageScale.ScaleY = 1;

                IndexCargo1Image.Source = new BitmapImage(new Uri("Images/Red_Cargo.png", UriKind.Relative));
                IndexCargo2Image.Source = new BitmapImage(new Uri("Images/Red_Cargo.png", UriKind.Relative));
                IntakeCargoImage.Source = new BitmapImage(new Uri("Images/Red_Cargo.png", UriKind.Relative));
                IntakeCargoTransfer.Source = new BitmapImage(new Uri("Images/Red_Cargo.png", UriKind.Relative));
                VerticalCargoTransfer.Source = new BitmapImage(new Uri("Images/Red_Cargo.png", UriKind.Relative));

                IndexSpot1SB.ActiveBrush = (Brush)this.Resources["IndexRedEnabled"];
                IndexSpot2SB.ActiveBrush = (Brush)this.Resources["IndexRedEnabled"];
                IntakeSpotSB.ActiveBrush = (Brush)this.Resources["IndexRedEnabled"];
            } else
            {
                HoundHoundingTargetSB.ActiveBrush = (Brush)this.Resources["HoundBlueHounding"];
                HoundHoundingTargetSB.InactiveBrush = (Brush)this.Resources["HoundBlueDefault"];
                HoundHoundingTargetSB.BorderBrush = (Brush)this.Resources["HoundBlueBorder"];
                HoundSeesTargetSB.ActiveBrush = (Brush)this.Resources["HoundBlueHounding"];
                HoundSeesTargetSB.InactiveBrush = (Brush)this.Resources["HoundBlueDefault"];
                HoundSeesTargetSB.BorderBrush = (Brush)this.Resources["HoundBlueBorder"];

                FieldCargo.ImageSource = new BitmapImage(new Uri("Images/Blue_Cargo.png", UriKind.Relative));
                FieldImageScale.ScaleX = 1;
                FieldImageScale.ScaleY = -1;

                IndexCargo1Image.Source = new BitmapImage(new Uri("Images/Blue_Cargo.png", UriKind.Relative));
                IndexCargo2Image.Source = new BitmapImage(new Uri("Images/Blue_Cargo.png", UriKind.Relative));
                IntakeCargoImage.Source = new BitmapImage(new Uri("Images/Blue_Cargo.png", UriKind.Relative));
                IntakeCargoTransfer.Source = new BitmapImage(new Uri("Images/Blue_Cargo.png", UriKind.Relative));
                VerticalCargoTransfer.Source = new BitmapImage(new Uri("Images/Blue_Cargo.png", UriKind.Relative));

                IndexSpot1SB.ActiveBrush = (Brush)this.Resources["IndexBlueEnabled"];
                IndexSpot2SB.ActiveBrush = (Brush)this.Resources["IndexBlueEnabled"];
                IntakeSpotSB.ActiveBrush = (Brush)this.Resources["IndexBlueEnabled"];
            }
            
        }

        private void updatePowerUsage()
        {
            if (timesPowerUpdated >= targetNumberTimesPowerUpdated)
            {
                NetworkTable powerTable = CommandoDashNT.GetSubTable("PowerUsage");
                //Update all power ProgressBoxes and TextBoxes
                //Drive
                double driveFLCurrent = powerTable.GetEntry("driveFLCurrent").GetDouble(0);
                double driveFRCurrent = powerTable.GetEntry("driveFRCurrent").GetDouble(0);
                double driveRLCurrent = powerTable.GetEntry("driveRLCurrent").GetDouble(0);
                double driveRRCurrent = powerTable.GetEntry("driveRRCurrent").GetDouble(0);
                double averageDriveCurrent = Math.Round((driveFLCurrent + driveFRCurrent + driveRLCurrent + driveRRCurrent) / 4, 1);
                DrivePowerProgress.Value = DrivePowerProgress.Maximum - averageDriveCurrent;
                DrivePowerText.Text = averageDriveCurrent.ToString();

                //Shooter
                double shooterLCurrent = powerTable.GetEntry("shooterLCurrent").GetDouble(0);
                double shooterRCurrent = powerTable.GetEntry("shooterRCurrent").GetDouble(0);
                ShooterPowerProgress.Value = ShooterPowerProgress.Maximum - ((shooterLCurrent + shooterRCurrent) / 2);
                ShooterPowerText.Text = Math.Round((shooterLCurrent + shooterRCurrent) / 2, 1).ToString();

                //Intake
                double intakeCurrent = powerTable.GetEntry("intakeCurrent").GetDouble(0);
                IntakePowerProgress.Value = IntakePowerProgress.Maximum - intakeCurrent;
                IntakePowerText.Text = Math.Round(intakeCurrent, 1).ToString();

                //Index
                double rampCurrent = powerTable.GetEntry("rampCurrent").GetDouble(0);
                double verticalCurrent = powerTable.GetEntry("verticalCurrent").GetDouble(0);
                IndexPowerProgress.Value = IntakePowerProgress.Maximum - ((verticalCurrent + rampCurrent) / 2);
                IndexPowerText.Text = Math.Round(((verticalCurrent + rampCurrent) / 2), 1).ToString();

                //Transfer
                double transferLCurrent = powerTable.GetEntry("transferLCurrent").GetDouble(0);
                double transferRCurrent = powerTable.GetEntry("transferRCurrent").GetDouble(0);
                TransferPowerProgress.Value = TransferPowerProgress.Maximum - ((transferRCurrent + transferLCurrent) / 2);
                TransferPowerText.Text = Math.Round((transferRCurrent + transferLCurrent) / 2, 1).ToString();

                //VRM
                double vrmCurrent = powerTable.GetEntry("VRMCurrent").GetDouble(0);
                VRMPowerProgress.Value = VRMPowerProgress.Maximum - vrmCurrent;
                VRMPowerText.Text = Math.Round(vrmCurrent, 1).ToString();

                //Overall Current and Battery Voltage
                TotalCurrentSB.Text = Math.Round(powerTable.GetEntry("totalCurrent").GetDouble(0), 1).ToString();
                BatteryVolatgeSB.Text = Math.Round(powerTable.GetEntry("batteryVoltage").GetDouble(0), 2).ToString();

                timesPowerUpdated = 0;
            } else
            {
                timesPowerUpdated++;
            }
        }

        private void updateGyroInfo()
        {
            double currAngle = CommandoDashNT.GetSubTable("SensorData").GetEntry("gyroAngle").GetDouble(0);
            GyroAngleEllipseAngle.Angle = currAngle + 45;
            GyroAngleText.Text = Math.Round(currAngle, 0).ToString() + "°";
            if (currAngle < 0)
            {
                Thickness newMargin = GyroAngleText.Margin;
                newMargin.Left = 41;
                GyroAngleText.Margin = newMargin;
            } else
            {
                Thickness newMargin = GyroAngleText.Margin;
                newMargin.Left = 45;
                GyroAngleText.Margin = newMargin;
            }

        }

        private void updateRobotPosition()
        {
            Thickness robotMargin = RobotRectangle.Margin;
            double robotXMeters = ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("Robot").GetDoubleArray(new double[3] { 0, 0, 0 })[0];
            double robotYMeters = ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("Robot").GetDoubleArray(new double[3] { 0, 0, 0 })[1];
            double robotRDegrees = ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("Robot").GetDoubleArray(new double[3] { 0, 0, 0 })[2];
            robotMargin.Left = (robotXMeters * (Field.Width / 16.4592)) - robotXFieldOffset;
            robotMargin.Top = (robotYMeters * (Field.Height / 8.2296)) - robotYFieldOffset;
            RobotRectangle.Margin = robotMargin;
            RobotRotateTransform.Angle = robotRDegrees;

            RobotXText.Text = Math.Round(robotXMeters,4).ToString() + "m";
            RobotYText.Text = Math.Round(robotYMeters,4).ToString() + "m";
            RobotRText.Text = Math.Round(robotRDegrees,4).ToString() + "°";
        }

        private void updateCargoPosition()
        {
            Thickness cargoMargin = CargoRectangle.Margin;
            double cargoXMeters = ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("SeenCargo").GetDoubleArray(new double[3] { 0, 0, 0 })[0];
            double cargoYMeters = ntInst.GetTable("SmartDashboard").GetSubTable("Field").GetEntry("SeenCargo").GetDoubleArray(new double[3] { 0, 0, 0 })[1];
            cargoMargin.Left = (cargoXMeters * (Field.Width / 16.4592)) - 15;
            cargoMargin.Top = (cargoYMeters * (Field.Height / 8.2296)) - 15;
            CargoRectangle.Margin = cargoMargin;

            CargoXText.Text = Math.Round(cargoXMeters, 4).ToString() + "m";
            CargoYText.Text = Math.Round(cargoYMeters, 4).ToString() + "m";
        }

        private void updateIndexSpot1()
        {
            Boolean spotHasCargo = CommandoDashNT.GetSubTable("IndexStatus").GetEntry("verticalSensor").GetBoolean(false);
            if (spotHasCargo)
            {
                IndexCargo1Image.Visibility = Visibility.Visible;
            } else
            {
                IndexCargo1Image.Visibility = Visibility.Hidden;
            }
            IndexSpot1SB.IsActive = spotHasCargo;
        }

        private void updateIndexSpot2()
        {
            Boolean spotHasCargo = CommandoDashNT.GetSubTable("IndexStatus").GetEntry("rampSensor").GetBoolean(false);
            if (spotHasCargo)
            {
                IndexCargo2Image.Visibility = Visibility.Visible;
            }
            else
            {
                IndexCargo2Image.Visibility = Visibility.Hidden;
            }
            IndexSpot2SB.IsActive = spotHasCargo;
        }

        private void updateIntakeSpot()
        {
            Boolean spotHasCargo = CommandoDashNT.GetSubTable("IndexStatus").GetEntry("entranceSensor").GetBoolean(false);
            if (spotHasCargo)
            {
                IntakeCargoImage.Visibility = Visibility.Visible;
            }
            else
            {
                IntakeCargoImage.Visibility = Visibility.Hidden;
            }
            IntakeSpotSB.IsActive = spotHasCargo;
        }

        private void updateVectorMapBox()
        {
            double targetRPM = Math.Round(CommandoDashNT.GetSubTable("SensorData").GetEntry("targetRPM").GetDouble(0), 0);
            string vectorMapRange = CommandoDashNT.GetSubTable("SensorData").GetEntry("vectorMapRange").GetString("0.0 - 0.0");
            VectorMapBox.Text = targetRPM + " | " + vectorMapRange;
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
            cameraURI = "http://roborio-5889-frc.local:1811/?action=stream";
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

        private void TaxiRadio_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("Taxi");
            TaxiRadioAL.IsChecked = true;
        }

        private void SpareRadio_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("Spare");
            SpareRadioAL.IsChecked = true;
        }

        private void IdealRadio_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("IdealAuto");
            IdealRadioAL.IsChecked = true;
            AutoLayoutImage.Source = new BitmapImage(new Uri("Images/FRC 2022 PathPlanner Field - Ideal.png", UriKind.Relative));
            AutoLayoutImageAL.Source = new BitmapImage(new Uri("Images/FRC 2022 PathPlanner Field - Ideal.png", UriKind.Relative));
        }

        private void DoubleShotRadio_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("DoubleShot");
            SecondRadioAL.IsChecked = true;
            AutoLayoutImage.Source = new BitmapImage(new Uri("Images/DoubleShotField.png", UriKind.Relative));
            AutoLayoutImageAL.Source = new BitmapImage(new Uri("Images/DoubleShotField.png", UriKind.Relative));

        }

        private void FullSendRadio_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("FullSend");
            FullSendRadioAL.IsChecked = true;
        }

        private void TaxiRadioAL_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("Taxi");
            TaxiRadio.IsChecked = true;
        }

        private void SpareRadioAL_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("Spare");
            SpareRadio.IsChecked = true;
        }

        private void IdealRadioAL_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("IdealAuto");
            IdealRadio.IsChecked = true;
            AutoLayoutImage.Source = new BitmapImage(new Uri("Images/FRC 2022 PathPlanner Field - Ideal.png", UriKind.Relative));
            AutoLayoutImageAL.Source = new BitmapImage(new Uri("Images/FRC 2022 PathPlanner Field - Ideal.png", UriKind.Relative));
        }

        private void DoubleShotRadioAL_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("DoubleShot");
            SecondRadio.IsChecked = true;
            AutoLayoutImage.Source = new BitmapImage(new Uri("Images/DoubleShotField.png", UriKind.Relative));
            AutoLayoutImageAL.Source = new BitmapImage(new Uri("Images/DoubleShotField.png", UriKind.Relative));
        }

        private void FullSendRadioAL_Clicked(object sender, RoutedEventArgs e)
        {
            NetworkTableEntry entry = CommandoDashNT.GetEntry("autoSelection");
            entry.SetString("FullSend");
            FullSendRadio.IsChecked = true;
        }

        private void updateShooterPIDBtn_Click(object sender, RoutedEventArgs e)
        {
            PIDTuningNT.GetEntry("shooterP").SetDouble(Double.Parse(shooterPBox.Text));
            PIDTuningNT.GetEntry("shooterD").SetDouble(Double.Parse(shooterDBox.Text));
            PIDTuningNT.GetEntry("shooterKs").SetDouble(Double.Parse(shooterKsBox.Text));
            PIDTuningNT.GetEntry("shooterKv").SetDouble(Double.Parse(shooterKvBox.Text));
        }

        private void updateHoundXPIDBtn_Click(object sender, RoutedEventArgs e)
        {
            PIDTuningNT.GetEntry("houndXP").SetDouble(Double.Parse(houndXPBox.Text));
            PIDTuningNT.GetEntry("houndXD").SetDouble(Double.Parse(houndXDBox.Text));
        }

        private void updateHoundYPIDBtn_Click(object sender, RoutedEventArgs e)
        {
            PIDTuningNT.GetEntry("houndYP").SetDouble(Double.Parse(houndYPBox.Text));
            PIDTuningNT.GetEntry("houndYD").SetDouble(Double.Parse(houndYDBox.Text));
        }

        private void updateHoundRPIDBtn_Click(object sender, RoutedEventArgs e)
        {
            PIDTuningNT.GetEntry("houndRP").SetDouble(Double.Parse(houndRPBox.Text));
            PIDTuningNT.GetEntry("houndRD").SetDouble(Double.Parse(houndRDBox.Text));
        }
    }
}
