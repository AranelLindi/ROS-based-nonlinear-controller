/*
  Stefan Lindörfer, Julian Balling
  20.08.2020
*/

/// Standardbibliotheken
#include <string>  // Stringverarbeitung
#include <memory>  // SmartPointer
#include <fstream> // nur zu Debugging
#include <cstdint> // int-Typedefs

/// ROS
#include <ros/ros.h>                  // ROS-Header
#include <tf/tf.h>                    // Transformationen
#include <tf/transform_broadcaster.h> // Senden der Transformationen zwischen Frames
#include <nav_msgs/Odometry.h>        // für publishen in world-Topic

/// Makros
//#define debug // entkommentieren wenn genauere Konsolenausgabe gewünscht wird

// Erstellt ein der Odometrie übergeordnetes Frame, mit der Startposition des Systems als Ursprung, das Transformationen veröffentlicht und ein world-Topic anlegt, mit dem erstellten Koordinatensystem als Grundlage.
class WorldFrameOdom
{
    /// Globale Variablen
    ros::Publisher pub;  // für world-Topic
    ros::Subscriber sub; // um Odometrie zu abonnieren

    double _x0{0.0}, _y0{0.0}, yaw0{0.0}; // Speichern aktuelle Koordinaten/Winkel des Ursprungs (werden kalibriert falls erforderlich).
    tf::Quaternion quat0;                 // Quaternion, dass Rotation zum Kalibrierungszeitpunkt darstellt.
    double _xold{0.0}, _yold{0.0};        // Werte der vorherigen Iteration (werden zum Vergleich und ggf. zur Neu-Kalibrierung des Ursprungs gespeichert).
    bool calibrated{false};               // Legt fest ob eine Neu-Kalibrierung des Ursprungs nötig ist.
    uint64_t counter{0};                  // Zählt Callback-Aufrufe (entspricht somit der Anzahl der Positionsänderung mit der Frequenz als Abtastung!)

public:
    void odomcallback(const boost::shared_ptr<const nav_msgs::Odometry> &msg) // wird bei Positionsänderung der Odometrie aufgerufen!
    {
        // ##############################################
        //          GRUNDSÄTZLICHES & VARIABLEN
        // ##############################################
        // nötig um Transformation in world-Frame zu senden:
        static tf::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        // Neue Werte für Iteration referenzieren:
        const double &x{msg->pose.pose.position.x};
        const double &y{msg->pose.pose.position.y};

        // tf::Quaternionen sind besser für mathematische Operationen als geometry_msgs::Quaternion, deshalb hier umwandeln:
        tf::Quaternion quat_cur;                                     // aktuelle Rotationsposition!
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat_cur); // "quat" wird by Reference verändert!

        // ##############################################
        //          KALIBRIERUNG KOORDINATENSYSTEM
        // ##############################################
        // prüfen ob zwischen zwei folgenden Iterationen größere Sprünge in den Koordinaten vorkommen, dann neu kalibrieren und Ursprung auf neue Position verschieben:
        if (fabs(x - _xold) > 1 || fabs(y - _yold) > 1)
            calibrated = false;

        // (Neu-)Kalibrierung des Ursprungs
        if (!calibrated)
        {
            // Koordinaten des Ursprungs im odom_combined-Frame speichern:
            _x0 = x;
            _y0 = y;

            // Ursprungsrotation bei jeder (Neu-)Kalibrierung speichern: (quat0 wird per Reference geändert!)
            tf::quaternionMsgToTF(msg->pose.pose.orientation, quat0);
            quat0 = quat0.normalize();

            yaw0 = tf::getYaw(quat0);

            // Neukalibrierung ohne Grund verhindern:
            calibrated = true;

            ROS_INFO("\n********************************\n(Neu-)Kalibrierung:\n\t\x1B[1mx0 = [%3f]\n\ty0 = [%3f]\n\tyaw0 = [%2f]\x1B[0m\n********************************", _x0, _y0, yaw0);
        }

        // ##############################################
        //          TRANSFORMATION UND tf-TOPIC
        // ##############################################
        // Übersetzungsobjekt von odom_combined- in world-Frame:
        tf::Transform transform;

        // Ortsvektor um z-Achse (yaw) mit dem negativen Startwinkel drehen: (entspricht dann einmal in world-Frame yaw = 0.0)
        transform.setOrigin(tf::Vector3(x - _x0, y - _y0, 0.0).rotate(tf::Vector3(0.0, 0.0, 1), -yaw0));

        // Aktuelle Rotation relativ zur Ursprungsrotation setzen:
        transform.setRotation(quat0.inverse() * quat_cur); // für den Fall, dass auf Rotation verzichtet werden soll: Ortsvektor nicht drehen(!) und 'transform.setRotation(quat_cur.inverse());' setzen! (hat funktioniert!)

        // Zeit, Ziel- und Quellframe festlegen: (Terminalbefehl: $ rosrun tf view_tf -und ggf.- $ dot -Tpng frames.gv -o frames.png)
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";        // fest!
        transformStamped.child_frame_id = "odom_combined"; // fest!

        // Berechnung der neuen Position (Koordinaten) und einfügen in transformStamped:
        tf::transformTFToMsg(transform, transformStamped.transform);

        // Berechnung der Rotation und einfügen in transformStamped:
        tf::quaternionTFToMsg(transform.getRotation(), transformStamped.transform.rotation);

        counter++; // Zähler inkrementieren

        // Senden der Transformation:
        br.sendTransform(transformStamped);

        // ##############################################
        //          PUBLISHEN IN world-TOPIC
        // ##############################################
        // Vorbereitung des veröffentlichens im entsprechenden Topic: (Variablen von oben werden auch hier verwendet!)
        nav_msgs::Odometry odom;

        odom.header.frame_id = "world";
        odom.child_frame_id = "odom_combined";
        odom.header.stamp = ros::Time::now();

        // Position in odom einfügen:
        tf::Pose pose{transform}; // zur Info: 'typedef tf::Transform tf::Pose' !
        tf::poseTFToMsg(pose, odom.pose.pose);

        // Rotation der Transformation in odom einfügen:
        tf::quaternionTFToMsg(transform.getRotation(), odom.pose.pose.orientation);

        odom.pose.covariance = msg->pose.covariance; // Kovarianz aus Parameterobjekt übernehmen!

        // veröffentlichen der Position im world-Topic:
        pub.publish(odom);

#ifdef debug
        ROS_INFO("%s: x = [%f]\ty = [%f]\tdyaw = [%f]", "worldframe_odom", transformStamped.transform.translation.x, transformStamped.transform.translation.y, dyaw);
#endif

        // #################################################################
        // Werte für nächste Iteration speichern: (um vergleichen zu können)
        _xold = x;
        _yold = y;
    }

    WorldFrameOdom() // Konstruktor
    {
        ros::NodeHandle n;

        // world-Topic einrichten und für publishen vorbereiten:
        pub = n.advertise<nav_msgs::Odometry>("world", 500, true);

        // Odometrie abonnieren und in callback behandeln:
        sub = n.subscribe("odom", 500, &WorldFrameOdom::odomcallback, this);
    }

    ~WorldFrameOdom() // Destruktor
    {
        // ROS-Objekte zerstören:
        pub.~Publisher();
        sub.~Subscriber();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "worldframe_odom");

    // Instanz von WorldFrame anlegen:
    const auto wf = std::make_unique<WorldFrameOdom>();

    ros::spin();

    return 0;
}; // wf wird hier zerstört & Destruktor aufgerufen!