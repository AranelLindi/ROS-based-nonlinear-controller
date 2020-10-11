/*
  Stefan Lindörfer, Julian Balling
  20.08.2020
*/

/// Standardbibliotheken
#include <fstream> // für Protokollierung der Positionen in eine Datei
#include <memory>  // SmartPointer

/// ROS
#include <ros/ros.h>								 // ROS Main Header
#include <ros/package.h>							 // um Paketpfad zu bekommen
#include <tf/transform_listener.h>					 // Benachrichtigungen über Transformationen in world-Frame
#include <geometry_msgs/PoseWithCovarianceStamped.h> // nötig für AMCL-Callback-Funktion! Wenn die nicht benötigt wird, include entfernen!
#include <nav_msgs/Odometry.h>						 // nötig für world-Topic-Callback Funktion

/// Makros
//#define debug // entkommentieren für ausführlichere Konsolenausgabe
//#define AMCL  // entkommentieren wenn mit AMCL statt world-Topic Pfad abgespeichert werden soll

class AMCLPoseProtocol
{
	/// Globale Variablen
	std::fstream fprotocol;																		  // Protokollierung der Position
	const std::string filename = (ros::package::getPath("custom_nodes")) + "/amcl_positions.dat"; // Pfad Ausgabedatei
	uint64_t counter{0};																		  // Zählt Iterationen

	// ROS Objekte
	ros::NodeHandle n;
	ros::Subscriber sub;

public:
#ifdef AMCL
	void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
	{
		// Referenzen auf neue Werte:
		const double &x{msg->pose.pose.position.x};
		const double &y{msg->pose.pose.position.y};

#ifdef debug
		ROS_INFO("%s: x = [%f]\ty = [%f]", "amclposition", x, y);
#endif

		// Koordinaten speichern:
		fprotocol << x << ' ' << y << '\n'; // nicht jedes Mal flushen, ist zu teuer!

		// nur alle 100 Schritte flushen:
		if (counter % 100 == 0)
			fprotocol << std::flush;

		counter++;
	}
#else
	void callback(const nav_msgs::Odometry::ConstPtr &msg)
	{
		// Referenzen auf neue Werte:
		const double &x{msg->pose.pose.position.x};
		const double &y{msg->pose.pose.position.y};

#ifdef debug
		ROS_INFO("%s: x = [%f]\ty = [%f]", "amclposition", x, y);
#endif

		// Koordinaten speichern:
		fprotocol << x << ' ' << y << '\n'; // nicht jedes Mal flushen, ist zu teuer!

		// nur alle 100 Schritte flushen:
		if (counter % 100 == 0)
			fprotocol << std::flush;

		counter++;
	}
#endif

	AMCLPoseProtocol() // Konstruktor
	{
#ifdef debug
		ROS_INFO("Dateipfad AMCL: %s", filename.c_str());
#endif

		// Protokollierungsdatei öffnen und als Ausgang markieren:
		fprotocol.open(filename.c_str(), std::ios::out); // ios::out überschreibt vorherige Daten! (gewünschtes Verhalten)

		// Konsolenausgabe:
		ROS_INFO("\n\n\t\x1B[33m\x1B[1m##### custom_nodes amclposition started! #####\x1B[0m\n");

		// entsprechendes Topic abonnieren:
#ifdef AMCL
		sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 100, &AMCLPoseProtocol::callback, this);
#else
		sub = n.subscribe<nav_msgs::Odometry>("world", 100, &AMCLPoseProtocol::callback, this);
#endif
	}

	~AMCLPoseProtocol() // Destruktor
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
	ros::init(argc, argv, "amclposition");

	// neue Instanz der Protokollierungsklasse anlegen:
	const auto amclpose = std::make_unique<AMCLPoseProtocol>();

	ros::spin(); // Node aufrechterhalten bis Strg+C

	return 0;
}