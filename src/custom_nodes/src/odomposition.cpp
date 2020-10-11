/*
  Stefan Lindörfer, Julian Balling
  20.08.2020
*/

/// Standardbibliotheken
#include <fstream> // für Protokollierung
#include <memory>  // SmartPointer

/// ROS
#include <ros/ros.h>			   // ROS-Header
#include <ros/package.h>		   // um Paketpfad zu bekommen
#include <tf/transform_listener.h> // veröffentlichte Transformationen abfragen
#include <nav_msgs/Odometry.h>	   // nötig für Odometrie-Callback-Funktion! Wenn die nicht benötigt wird, include entfernen!

/// Makros
//#define debug // entkommentieren für ausführlichere Konsolenausgabe
//#define ODOM  // entkommentieren wenn world-Topic für Pfadextraktion verwendet werden soll

class OdomPoseProtocol
{
	/// Globale Variablen
	std::fstream fprotocol;																		  // Ausgabestream
	const std::string filename = (ros::package::getPath("custom_nodes")) + "/odom_positions.dat"; // Pfad Ausgabedatei
	uint64_t counter{0};
	
	// ROS Objekte:
	ros::NodeHandle n;
	ros::Subscriber sub;																		  // Zählt Iterationen

public:
	void callback(const nav_msgs::Odometry::ConstPtr &msg)
	{
		// Referenzen auf Positionswerte:
		const double &x{msg->pose.pose.position.x};
		const double &y{msg->pose.pose.position.y};

#ifdef debug
		ROS_INFO("%s: x = [%f]\ty = [%f]", "odomposition", x, y);
#endif

		fprotocol << x << ' ' << y << '\n'; // nicht jedes Mal flushen! Ist zu teuer!

		// Alle 100 Schritte flushen:
		if (counter % 100 == 0)
			fprotocol << std::flush;

		counter++;
	}

	OdomPoseProtocol() // Konstruktor
	{
#ifdef debug
		ROS_INFO("Dateipfad Odometrie: %s", filename.c_str());
#endif

		// Datei mit Positionsdaten für Ausgabe öffnen
		fprotocol.open(filename.c_str(), std::ios::out); // ios::out überschreibt vorherige Einträge! (gewünschtes Verhalten)

		// Konsolenausgabe:
		ROS_INFO("\n\n\t\x1B[33m\x1B[1m##### custom_nodes odomposition started! #####\x1B[0m\n");

#ifdef ODOM
		sub = n.subscribe<nav_msgs::Odometry>("odom", 100, &OdomPoseProtocol::callback, this);
#else
		sub = n.subscribe<nav_msgs::Odometry>("world", 100, &OdomPoseProtocol::callback, this);
#endif
	}

	~OdomPoseProtocol() // Destruktor
	{
		// Nach Abbruch: Stream schließen & Node beenden:
		fprotocol.flush();
		fprotocol.close();
		fprotocol.clear();
	}
};

int main(int argc, char **argv)
{
	// Beim ROS Master anmelden:
	ros::init(argc, argv, "odomposition");

	// OdomPoseProtocol instanzieren:
	const auto odom_pose = std::make_unique<OdomPoseProtocol>();

	ros::spin(); // Node aufrechterhalten bis Strg+C

	return 0;
}