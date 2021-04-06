using EventArgsLibrary;
using System;
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
            Recule,
        }

        Trajectory trajectory = Trajectory.Rotation;
        
        float Fech = 50f;
        
        int robotId;

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

            wayPointLocation = new Location(1, 1, 0, 0, 0, 0);

            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 0.5;// 2; //en m.s-2
            accelAngulaire = 0.5 * Math.PI * 1.0; //en rad.s-2

            vitesseLineaireMax = 2; //1; //en m.s-1               
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
        
        public void OnWaypointReceived(object sender, PositionArgs destination)
        {
            /// Mise à jour du waypoint courant
            wayPointLocation.X = destination.X;
            wayPointLocation.Y = destination.Y;

            /// Initialisation de la machine à état de déplacement du Ghost
        }

        void CalculateGhostPosition()
        {

            switch (trajectory)
            {
                case Trajectory.Attente:

                    //ghostLocationRefTerrain.Vlin = 0; 
                    //Console.WriteLine( "vitesse : " + ghostLocationRefTerrain.Vlin);

                    break;

                case Trajectory.Rotation:

                    double thetaGhostArret = ghostLocationRefTerrain.Vtheta * ghostLocationRefTerrain.Vtheta / (2 * accelAngulaire);
                    double thetaGhostCible = Math.Atan2((wayPointLocation.Y - currentLocationRefTerrain.Y), (wayPointLocation.X - currentLocationRefTerrain.X));
                    double thetaGhostRestant = thetaGhostCible - Toolbox.ModuloByAngle(thetaGhostCible, ghostLocationRefTerrain.Theta);

                    if (thetaGhostArret < Math.Abs(thetaGhostRestant))
                    {


                        if (Math.Abs(ghostLocationRefTerrain.Vtheta) < vitesseAngulaireMax)
                        {
                            if (thetaGhostRestant > 0)
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

                    ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta * 1 / Fech;

                    if (thetaGhostRestant < Toolbox.DegToRad(0.2))
                    {
                        trajectory = Trajectory.Avance;
                    }

                    break;

                case Trajectory.Avance:


                    double proScalaire = ghostLocationRefTerrain.X * wayPointLocation.X + ghostLocationRefTerrain.Y * wayPointLocation.Y ;   // ghost scalaire cible
                    double normeCibleSquare = wayPointLocation.X * wayPointLocation.X + wayPointLocation.Y * wayPointLocation.Y ;       // norme de cible au carre 
                    double projeteX = proScalaire / normeCibleSquare * wayPointLocation.X;      // posX du projete  
                    double projeteY = proScalaire / normeCibleSquare * wayPointLocation.Y;      // posY du projete 
                    double dprojete = Math.Sqrt(projeteX * projeteX + projeteY * projeteY);      // distance du projete 
                    //double vGhostLin = Math.Sqrt(ghostLocationRefTerrain.Vy * ghostLocationRefTerrain.Vy + ghostLocationRefTerrain.Vx * ghostLocationRefTerrain.Vx);

                    double dCible = Math.Sqrt(normeCibleSquare);
                    double dGhostRestant = dCible - dprojete;
                    double dGhostArret = (ghostLocationRefTerrain.Vlin * ghostLocationRefTerrain.Vlin) / (2 * accelLineaire); //(vGhostLin * vGhostLin) / (2 * accelLineaire);
                    //vGhostLin est remplace par ghostLocationRefTerrain.Vx

                    
                    if (dGhostArret *2 < dGhostRestant )
                        {
                        // on accélère
                        if (ghostLocationRefTerrain.Vlin < vitesseLineaireMax)
                        {
                            ghostLocationRefTerrain.Vlin += accelLineaire / Fech;
                                //ghostLocationRefTerrain.X += (vGhostLin * Math.Cos(wayPointLocation.Theta)) / Fech;
                                //ghostLocationRefTerrain.Y += (vGhostLin * Math.Sin(wayPointLocation.Theta)) / Fech;

                                //if(dGhostArret > dGhostRestant)
                                //{
                                //    vGhostLin -= accelLineaire / Fech;
                                //}
                        }
                        else
                        {
                                //if (dGhostArret > dGhostlin)
                                //{
                                //    vGhostlin -= accelLineaire / Fech;
                                //}
                        }
                        }
                    else 
                    {
                        ghostLocationRefTerrain.Vlin -= accelLineaire / Fech;
                            //ghostLocationRefTerrain.X += (vGhostLin*Math.Cos(wayPointLocation.Theta)) / Fech;
                            //ghostLocationRefTerrain.Y += (vGhostLin *Math.Sin(wayPointLocation.Theta)) / Fech;
                    }

                    ghostLocationRefTerrain.X +=  Math.Cos(ghostLocationRefTerrain.Theta)* ghostLocationRefTerrain.Vlin / Fech;
                    ghostLocationRefTerrain.Y +=  Math.Sin(ghostLocationRefTerrain.Theta)* ghostLocationRefTerrain.Vlin / Fech;
                    

                    Console.WriteLine(" Vitesse : " + ghostLocationRefTerrain.Vlin);

                    if (dGhostRestant < 0.001)
                    {
                        trajectory = Trajectory.Attente;
                    }




                    break;

                case Trajectory.Recule: 




                    break;
            }

            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
        }

        void PIDPosition()
        {
            //A remplir
            double vLineaireRobot=0, vAngulaireRobot=0;
            
            /// On envoie les vitesses consigne.
            /// Indispensable en permanence, sinon la sécurité de l'embarqué reset le contrôle moteur
            /// en l'absence d'orde pendant 200ms
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

        public event EventHandler<StringEventArgs> OnTextMessageEvent;
        public virtual void OnTextMessage(String message)
        {
            OnTextMessageEvent?.Invoke(this, new StringEventArgs { value=message });
        }
    }
}
