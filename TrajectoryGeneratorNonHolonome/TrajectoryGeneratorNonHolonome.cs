﻿using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace TrajectoryGeneratorNonHolonomeNS
{
    public class TrajectoryGeneratorNonHolonome
    {
        public enum Trajectory
        {
            Attente,
            Rotation,
            Avance,  
        }

        Trajectory trajectory = Trajectory.Rotation;


        float Fech = 50f;
        
        

        int robotId;

        double samplingFreq;

        Location currentLocationRefTerrain;
        Location wayPointLocation;
        Location ghostLocationRefTerrain;

        double accelLineaire, accelAngulaire;
        double vitesseLineaireMax, vitesseAngulaireMax;

        AsservissementPID PID_Position_Lineaire;
        AsservissementPID PID_Position_Angulaire;

        public TrajectoryGeneratorNonHolonome(int id)
        {
            robotId = id;
            InitRobotPosition(0, 0, 0);
            InitPositionPID();

            wayPointLocation = new Location(1, 0.5, 0, 0, 0, 0);

            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 0.5; //en m.s-2
            accelAngulaire = 0.5 * Math.PI * 1.0; //en rad.s-2

            vitesseLineaireMax = 1; //en m.s-1               
            vitesseAngulaireMax = 1 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(20.0, 10.0, 0, 100, 100, 1);
            PID_Position_Angulaire = new AsservissementPID(20.0, 10.0, 0, 5 * Math.PI, 5 * Math.PI, Math.PI);
        }

        public void InitRobotPosition(double x, double y, double theta)
        {
            Location old_currectLocation = currentLocationRefTerrain;
            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            wayPointLocation = new Location(x, y, theta, 0, 0, 0);
            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            PIDPositionReset();
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {
            if (robotId == e.RobotId)
            {
                currentLocationRefTerrain = e.Location;
                CalculateGhostPosition();
                PIDPosition();
            }
        }

        void CalculateGhostPosition()
        {
            
            switch(trajectory)
            {
                case Trajectory.Attente :


                    break;

                case Trajectory.Rotation:
                    
                    double thetaGhostArret = ghostLocationRefTerrain.Vtheta * ghostLocationRefTerrain.Vtheta / ( 2* accelAngulaire);
                    double thetaGhostCible = Math.Atan2((wayPointLocation.Y - currentLocationRefTerrain.Y), (wayPointLocation.X - currentLocationRefTerrain.X));
                    double thetaGhostRestant = thetaGhostCible - Toolbox.ModuloByAngle(thetaGhostCible, ghostLocationRefTerrain.Theta);

                    if (thetaGhostArret < Math.Abs(thetaGhostRestant))
                    {

                        
                        if (Math.Abs(ghostLocationRefTerrain.Vtheta) < vitesseAngulaireMax)
                        {
                            if (thetaGhostRestant>0)
                            {
                                ghostLocationRefTerrain.Vtheta += accelAngulaire * 1 / Fech;
                            }
                            else
                            {
                                ghostLocationRefTerrain.Vtheta -= accelAngulaire * 1 / Fech;
                            }
                        }
                        else
                        {
                            //rien
                        }
                    }

                    else
                    {
                        if (thetaGhostRestant > 0)
                        {
                            ghostLocationRefTerrain.Vtheta -= accelAngulaire * 1 / Fech;
                        }
                        else
                        {
                            ghostLocationRefTerrain.Vtheta += accelAngulaire * 1 / Fech;
                        }
                        
                    }

                    ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta * 1/  Fech;

                    if(thetaGhostRestant < Toolbox.DegToRad(0.2))
                    {
                        trajectory = Trajectory.Avance;
                    }
                    
                    break;

                case Trajectory.Avance:

                    Console.WriteLine("oui");
                    break;
            }

            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
        }

        void PIDPosition()
        {
            //A remplir
            double vLineaireRobot=0, vAngulaireRobot=0;


            //Si tout c'est bien passé, on envoie les vitesses consigne.
            OnSpeedConsigneToRobot(robotId, (float)vLineaireRobot, (float)vAngulaireRobot);
        }

        void PIDPositionReset()
        {
            if (PID_Position_Angulaire != null && PID_Position_Lineaire != null)
            {
                PID_Position_Lineaire.ResetPID(0);
                PID_Position_Angulaire.ResetPID(0);
            }
        }

        /*************************************** Outgoing Events ************************************/

        public event EventHandler<LocationArgs> OnGhostLocationEvent;
        public virtual void OnGhostLocation(int id, Location loc)
        {
            var handler = OnGhostLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = loc });
            }
        }

        public event EventHandler<PolarSpeedArgs> OnSpeedConsigneEvent;
        public virtual void OnSpeedConsigneToRobot(int id, float vLineaire, float vAngulaire)
        {
            var handler = OnSpeedConsigneEvent;
            if (handler != null)
            {
                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire});
            }
        }
    }
}
