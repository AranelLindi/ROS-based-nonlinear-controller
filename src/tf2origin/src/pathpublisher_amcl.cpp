/*
  Stefan Lindörfer, Julian Balling
  20.08.2020
*/

/// Standardbibliotheken
#include <memory> // SmartPointer

/// ROS
#include <ros/ros.h>								 // ROS-Header
#include <tf/tf.h>									 // Transformation
#include <nav_msgs/Path.h>							 // Pfad-Objekt (Topic)
#include <nav_msgs/Odometry.h>						 // für world-Topic
#include <geometry_msgs/PoseStamped.h>				 // Umwandlung in Callback's
#include <geometry_msgs/PoseWithCovarianceStamped.h> // für amcl_pose-Topic

/// Makros
//#define debug // entkommentieren für erweiterte Konsolenausgabe
//#define AMCL // entkommentieren wenn mit amcl_pose- statt world-Topic gearbeitet werden soll

// Wird in main instanziiert und übernimmt das Abonnieren und Publishen des gefahrenen Pfades mit AMCL Positionen als Grundlage.
class AMCLToPath
{
	/// Globale Variablen
	ros::Publisher pub;	 // veröffentlichen des Pfades als Topic
	ros::Subscriber sub; // abonnieren der AMCL Positionen in der Karte
	nav_msgs::Path path; // abgeleiteter std::vector, der Pfad enthält

public:
#ifdef AMCL
	void amclcallback(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr> &msg) // für AMCL!
	{
		// msg in PoseStamped umwandeln:
		geometry_msgs::PoseStamped pose;
		pose.header = msg->get()->header;
		pose.pose = msg->get()->pose.pose;

		// Rotation festlegen:
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(msg->get()->pose.pose.orientation));

		// PoseStamped in path hinzufügen:
		path.header = msg->get()->header;
		path.poses.push_back(pose);

#ifdef debug
		ROS_INFO("AMCL Path updated!");
#endif

		// veröffentlichen im Topic:
		pub.publish(path);
	}
#else
	void worldcallback(const boost::shared_ptr<const nav_msgs::Odometry> &msg) // für AMCL über WorldFrame!
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
		ROS_INFO("AMCL Path updated!");
#endif

		// veröffentlichen im Topic:
		pub.publish(path);
	}
#endif

	AMCLToPath() // Konstruktor
	{
		ros::NodeHandle n;

		// Vorbereiten des publishens über das path-Topic:
		pub = n.advertise<nav_msgs::Path>("path_amcl", 300, true);

		// ################
		// world-Frame abonnieren um Pfad aufzuzeichnen:
		// (falls stattdessen amcl_pose abonniert werden soll, "world" durch "amcl_pose" ersetzen und callback Funktion entsprechend anpassen! (Ist dann aber auch wieder map-Frame!))

#ifdef AMCL
		sub = n.subscribe("amcl_pose", 100, &AMCLToPath::amclcallback, this);
#else
		sub = n.subscribe("world", 100, &AMCLToPath::worldcallback, this);
#endif
		// ################
	}

	~AMCLToPath() // Destruktor
	{
		pub.~Publisher();
		sub.~Subscriber();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pathpublisher_amcl");

	// Instanziieren eines AMCLToPath-Objekts:
	const auto amcl = std::make_unique<AMCLToPath>();

	ros::spin(); // Node aufrechterhalten bis Strg+C

	return 0;
} // amcl wird hier zerstört & Destruktor aufgerufen