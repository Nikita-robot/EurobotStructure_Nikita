using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace Positioning2WheelsNS
{
    public class Positioning2Wheels
    {
        int robotId;
        Location PosRobotRefTerrain= new Location();
        double Fech = 50f;
        
        public Positioning2Wheels(int id)
        {
            robotId = id;
        }

        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            //double X = 0, Y = 0, theta = 0, X_1, Y_1, F = 50, theta_1;

            PosRobotRefTerrain.Theta += (float) e.Vtheta / Fech;

            PosRobotRefTerrain.X += (float) e.Vx / Fech * (float) Math.Cos(PosRobotRefTerrain.Theta);
            PosRobotRefTerrain.Y += (float) e.Vx / Fech * (float) Math.Sin(PosRobotRefTerrain.Theta);
            PosRobotRefTerrain.Vx = (float) e.Vx * (float) Math.Cos(PosRobotRefTerrain.Theta);
            PosRobotRefTerrain.Vy = (float) e.Vx * (float) Math.Sin(PosRobotRefTerrain.Theta);
            Console.WriteLine("X : "+ PosRobotRefTerrain.X+ "Y : "+ PosRobotRefTerrain.Y+ "theta : "+ PosRobotRefTerrain.Theta);

            OnCalculatedLocation(robotId, PosRobotRefTerrain);
        }

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(int id, Location locationRefTerrain)
        {
            var handler = OnCalculatedLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = locationRefTerrain });
            }
        }
    }
}
