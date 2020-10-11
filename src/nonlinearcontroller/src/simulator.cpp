/*
  Stefan Lindörfer, Julian Balling
  20.08.2020
*/

/// Standardbibliothek
#include <fstream> // Daten in Datei schreiben
#include <string>  // Stringverarbeitung
#include <memory>  // SmartPointer
#include <cstdint> // für int-Types

/// ROS
#include <ros/ros.h>	 // ROS-Header
#include <ros/package.h> // Zugriff auf Paket-Pfad

/// zur Verfügung gestellter Code
#include <nonlinearcontroller/gio_path.h> // Regler

/// Makros
#define PATH_SOURCE "Haus.dat" // ggf. ändern wenn anderer Pfad abgefahren werden soll! (Also andere Datei als Quelle)
#define PATH_DEST "protocol"   // ggf. ändern wenn in anderer Datei gespeichert werden soll!
//#define debug				   // entkommentieren für ausführlichere Debug-Ausgabe

/// Globale Variablen
// Quelle Pfad:
static const std::string filepath = (ros::package::getPath("nonlinearcontroller")) + "/src/data/" + PATH_SOURCE; // Datei mit Pfadkoordinaten öffnen und CGioController übergeben
// Ziel Ausgabe:
static const std::string filename_fpose = (ros::package::getPath("nonlinearcontroller")) + "/" + PATH_DEST + "_pose.dat";
static const std::string filename_fvphi = (ros::package::getPath("nonlinearcontroller")) + "/" + PATH_DEST + "_vphi.dat";
// Konstante für kleinste Zeiteinheit:
static const float dt{10e-3}; // 10 ms (0.01) // Abhängig von der Frequenz, die vom Controller benutzt wird!

// Ref.-Variablen: (werden von CGioContr.->getNextState(..) geändert!)
static double u{0.0}, w{0.0}, vleft{0.0}, vright{0.0};

// Werte und Änderungen für Positionen:
static double x{0.0}, y{0.0}, phi{0.0};
static double dx{0.0}, dy{0.0}, dphi{0.0};

// Gibt an ob Strecke abgefahren ist (false) oder Robo noch unterwegs ist (true)
static bool notFinished{true};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulator");

#ifdef debug
	ROS_INFO("Lese Pfad aus: %s", filepath.c_str());
#endif

	// CGioController (Regler) wird initialisiert:
	const auto controller = std::make_unique<CGioController>(); // unique_ptr

#ifdef debug
	ROS_INFO("Counter: %lu", counter);
#endif
	if (controller->getPathFromFile(filepath.c_str()))
	{

		ROS_INFO("\n\n\t\x1B[33m\x1B[1m##### SIMULATION GESTARTET! #####\x1B[0m\n");

#ifdef debug
		ROS_INFO("-- Einlesen der Pfaddatei erfolgreich --");
#endif

		// Erste Position abfragen (0, 0):
		controller->getPose(x, y, phi);

#ifdef debug
		ROS_INFO("Startposition:\n\tx = %f\n\ty = %f", x0, y0);
#endif

		// Streams die berechnete Daten schreiben:
		std::fstream fpose, fvphi;
		fpose.open(filename_fpose.c_str(), ios::out);
		fvphi.open(filename_fvphi.c_str(), ios::out);

		// Counter für Schleife
		uint64_t loop_counter{0};

		// Simulation starten:
		while (ros::ok() && notFinished) // notFinished muss 'false' sein (korrekt!), damit die Schleife abbricht!
		{
			// Nexte Position vorhersagen:
			notFinished = controller->getNextState(u, w, vleft, vright);

			// Änderungen berechnen
			dphi = dt * w;
			dx = u * cos(phi + dphi * 0.5) * dt;
			dy = u * sin(phi + dphi * 0.5) * dt;

#ifdef debug
			// Berechnete Werte anzeigen:
			ROS_INFO("u = %f; w = %f; vleft = %f; vright = %f", u, w, vleft, vright);
			ROS_INFO("dx = %f; dy = %f; dphi = %f", dx, dy, dphi);
			ROS_INFO("x = %f; y = %f; phi = %f", x, y, phi);
#endif

			// Änderungen zu bestehenden Werten addieren:
			x += dx;
			y += dy;
			phi += dphi;

#ifdef debug
			// Ausgabe für Benutzer:
			ROS_INFO("%li : x=%f / y=%f / phi=%f", counter, x, y, phi);
			ROS_INFO("--------------------------------------");
#endif

			// Neue Position setzen:
			controller->setPose(x, y, phi);

			// Daten in Stream schreiben: (durch Leerzeichen getrennt!)
			fpose << x << ' ' << y << '\n';
			fvphi << vleft << ' ' << vright << ' ' << fmod(fmod(phi, 2 * M_PI) + 2 * M_PI, 2 * M_PI) << '\n'; // Regler benutzt phi NICHT dphi!

			loop_counter++;
		}

		// Stream flushen & schließen:
		fpose.flush();
		fvphi.flush();

		fpose.close();
		fvphi.close();

		fpose.clear();
		fvphi.clear();

#ifdef debug
		ROS_INFO("Anzahl Iterationen: %lu", loop_counter);
#endif

		ROS_INFO("\n\n\t\x1B[32m\x1B[1m##### SIMULATION ABGESCHLOSSEN! #####\x1B[0m\n");
	}
	else
	{
		// Einlesen schlug fehl:
		ROS_ERROR("Einlesen der Pfaddatei fehlgeschlagen!");
		return 1;
	}

	return 0;
}