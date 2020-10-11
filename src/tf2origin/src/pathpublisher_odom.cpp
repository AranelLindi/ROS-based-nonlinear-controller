/*
  Stefan Lindörfer, Julian Balling
  20.08.2020
*/

/// Standardbibliothek
#include <memory> // SmartPointer

/// ROS
#include <ros/ros.h>		   // ROS-Header
#include <tf/tf.h>			   // Transformationen
#include <nav_msgs/Path.h>	   // Pfad-Objekt
#include <nav_msgs/Odometry.h> // Odometrie-Objekt

/// Makros
//#define debug // entkommentieren für erweiterte Konsolenausgabe
//#define ODOM // entkommentieren wenn mit odom- statt world-Topic gearbeitet werden soll

// Wird in main instanziiert und übernimmt das Abonnieren und Publishen des gefahrenen Pfade mit der Odometrie als Grundlage.
class OdomToPath
{
	ros::Publisher pub;	 // veröffentlichen des Pfades als Topic
	ros::Subscriber sub; // abonnieren der Odometrie
	nav_msgs::Path path; // abgeleiteter std::vector, der Pfad enthält

public:
	void odomcallback(const boost::shared_ptr<const nav_msgs::Odometry> &msg) // für Odometrie & world-Topic!
	{
		// msg in PoseStamped umwandeln:
		geometry_msgs::PoseStamped pose;
		pose.header = msg->header;
		pose.pose = msg->pose.pose;

		// Rotation festlegen:
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(msg->pose.pose.orientation));

		// PoseStamped in path hinzufügen:
		path.header = msg->header;
		path.poses.push_back(pose);

#ifdef debug
		ROS_INFO("Odometry Path updated!");
#endif

		// veröffentlichen im Topic:
		pub.publish(path);
	}

	OdomToPath() // Konstruktor
	{
		ros::NodeHandle n;

		// Vorbereiten des publishens über das path-Topic:
		pub = n.advertise<nav_msgs::Path>("path_odom", 300, true);

// world-Frame abonnieren um Pfad aufzuzeichnen: (falls statt dessen Odometrie abonniert werden soll, "world" durch "odom" ersetzen. (Ist dann jedoch auch wieder odom_combined-Frame!))
#ifdef ODOM
		sub = n.subscribe("odom", 100, &OdomToPath::odomcallback, this);
#else
		sub = n.subscribe("world", 100, &OdomToPath::odomcallback, this);
#endif
	}

	~OdomToPath() // Destruktor
	{
		pub.~Publisher();
		sub.~Subscriber();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pathpublisher_odom");

	// Instanziieren eines OdomToPath-Objekts:
	const auto odom = std::make_unique<OdomToPath>();

	ros::spin(); // Node aufrechterhalten bis Strg+C

	return 0;
} // odom wird hier zerstört & Destruktor aufgerufen!
