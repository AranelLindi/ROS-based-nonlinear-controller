/*
  Stefan Lindörfer, Julian Balling 
  20.08.2020
*/

/// Standardbibliothek
#include <string>  // Stringverarbeitung
#include <memory>  // SmartPointer
#include <cstdint> // int-Typedefs

/// ROS
#include <ros/ros.h>		   // ROS-Header
#include <tf/tf.h>			   // Transformationen
#include <nav_msgs/Odometry.h> // Odometrie Positionen
#include <ros/package.h>	   // Zugriff auf Package-Pfad

/// zur Verfügung gestellter Code
#include "volksbot/vels.h"				  // für Publishing in Vel-Topic (Geschwindigkeit)
#include "nonlinearcontroller/gio_path.h" // Regler

/// Makros
#define PATH_SOURCE "acht.dat"
//#define debug // entkommentieren für erweiterte Debug-Ausgabe

/// Globale Variablen
static const float freq{20.0};																					 // Frequenz in Hz
static const std::string filepath = (ros::package::getPath("nonlinearcontroller")) + "/src/data/" + PATH_SOURCE; // Datei mit Pfadkoordinaten öffnen und CGioController übergeben

// Geschwindigkeit: (Änderung per Reference vom Regler)
static double vleft{0.0}, vright{0.0};

// Variablen für Regelung:
static double u{0.0}, w{0.0};
static uint32_t loop{0};

// Regler:
static std::unique_ptr<CGioController> controller;

// Publisher für Geschwindigkeits-Topic & Container für Topic:
ros::Publisher vel_pub;
volksbot::vels velocity;

// Gibt an ob Pfad komplett abgefahren wurde (true):
static bool notFinished{true}; // auf 'false' falls Pfad abgefahren ist. Wichtig! Initialisieren mit 'true', da sonst while Schleife nur eine Iteration hat

// Maximalgeschwindigkeit für Pfadverfolgung:
static const float SPEED{-20.0};

// Odometrie Funktion: Bekommt Positionsänderungen über normale Odometrie
void positionchanged(const nav_msgs::Odometry::ConstPtr &msg)
{
	// msg in Pose umwandeln: (per Reference!)
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);

	// Referenz auf aktuelle Positionskoordinaten:
	const double &x{msg->pose.pose.position.x};
	const double &y{msg->pose.pose.position.y};
	// Rotationswinkel abfragen:
	const double phi{tf::getYaw(pose.getRotation())};

	// Neue Position an Regler übergeben...
	controller->setPose(x, y, phi);
	// ... und werte für nächste Iteration der while-Schleife (main) berechnen lassen:
	notFinished = controller->getNextState(u, w, vleft, vright);

#ifdef debug
	ROS_INFO("VelUpdate: vleft=[%f], vright=[%f]", velocity.left, velocity.right);
	ROS_INFO("Odom PoseUpdate: x=[%f], y=[%f], phi=[%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller_world");

	ros::NodeHandle n;

	// Vorbereitung für Publishen in "Vel" Topic:
	vel_pub = n.advertise<volksbot::vels>("Vel", 300, false);

	// Regler instanzieren (SmartPointer):
	controller = std::make_unique<CGioController>();

	// Abzufahrenden Pfad in Regler einlesen:
	if (!(controller->getPathFromFile(filepath.c_str())))
	{
		ROS_ERROR("Einlesen der Pfaddatei ist fehlgeschlagen!");
		return 1;
	}

	ROS_INFO("\n\n\t\x1B[33m\x1B[1m##### CONTROLLER GESTARTET! #####\x1B[0m\n");
	ROS_INFO("Pfaddatei: %s", filepath.c_str());

	// world-Topic abonnieren um Positionen im world-Frame zu bekommen:
	ros::Subscriber sub = n.subscribe("world", 500, positionchanged); // abonniert world-Topic

	// Robo auf Nullpunkt setzen:
	controller->setPose(0.0, 0.0, 0.0);

	// erste Regelanweisung abfragen:
	controller->getNextState(u, w, vleft, vright);

	uint64_t counter{0}; // Zählt Iterationen in while-Schleife

	ros::Rate rate(freq);

	while (ros::ok() && notFinished) // notFinished muss 'false' sein, damit die Schleife abbricht. (Funktioniert korrekt!)
	{
		// Neue Geschwindigkeiten festlegen: (werden per Reference von controller->getNextStatus(..) geändert!)
		velocity.left = SPEED * vleft;
		velocity.right = SPEED * vright;
		velocity.id = counter; // vermutlich nicht notwendig

#ifdef debug
		ROS_INFO("%lu : vleft=[%f]\tvright=[%f]", counter, velocity.left, velocity.right);
#endif

		// Geschwindigkeiten im Topic "Vel" publishen:
		vel_pub.publish(velocity);

		// auf einen callback warten:
		ros::spinOnce();

		counter++;

		// Eventuelle überschüssige Zeit bis zur nächsten Iteration (Frequenz!) warten:
		rate.sleep();
	}

	ROS_INFO("\n\n\t\x1B[32m\x1B[1m##### FAHRT ABGESCHLOSSEN! #####\x1B[0m\n");

	// Nach dem abfahren, anhalten:
	velocity.left = 0.0;
	velocity.right = 0.0;
	velocity.id = counter + 1;

	vel_pub.publish(velocity);

	return 0;
}