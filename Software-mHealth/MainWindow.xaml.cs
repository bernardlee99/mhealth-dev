using HidSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;
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

namespace mHealth
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        HidStream stream;
        public MainWindow()
        {
            InitializeComponent();
            //Look for devices of such VID,PID AND VID
            var devs = DeviceList.Local.GetHidDevices(0x0483, 0x5750).ToList();
            if(devs.Count != 1)
            {
                Console.WriteLine("Failed to discover device."); Environment.Exit(2);
            }
            var device = devs[0];
            if (!device.TryOpen(out stream)) { Console.WriteLine("Failed to open device."); Environment.Exit(2); }

            Timer myTimer = new Timer();
            myTimer.Elapsed += MyTimer_Elapsed;
            myTimer.Interval = 50;
            myTimer.Start();
        }

        private void MyTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            var bytes = new byte[64];
            int count = stream.Read(bytes, 0, bytes.Length);
            String str = System.Text.ASCIIEncoding.ASCII.GetString(bytes);
            String[] separator = {"|"};
            String[] split = str.Split(separator, 5, StringSplitOptions.None);

            Console.WriteLine(str);
            Console.WriteLine(split[0]);
            Console.WriteLine(split[1]);
            Console.WriteLine(split[2]);
            Console.WriteLine(split[3]);

            Dispatcher.Invoke(() =>
            {
                main_accel.Content = split[0];
                main_gyro.Content = split[1];
                main_mag.Content = split[2];
                main_ppg.Content = split[3];
            });
            
        }
    }
                
            
}
