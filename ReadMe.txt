Hardwarepraktikum - SS20

eigene Packages neben den mitgelieferten:
=========================================
- nonlinearcontroller	-- Enthält den Simulator sowie den Controller (einen für beide Verfahren)
- custom_nodes		-- Enthält Nodes um Pfade von Odometrie und AMCL zu speichern
- tf2origin		-- Enthält Nodes für world-Frame (Koordinatensystem) und Pfad-Publishing für RViz


Generelle Anpassungsmöglichkeiten:
==================================
Jeder Node enthält debug-Makro, dass entkommentiert werden kann um genauere Konsolenausgabe mit Zwischenergebnissen zu erhalten. (Ist bei allen aktuell auskommentiert)


Anpassungen für Controller & Simulator:
=======================================
Um Pfad für Controller/Simulator zu ändern, im Package 'nonlinearcontroller/src' die Datei controller_world.cpp -oder- simulator.cpp öffnen und das Makro PATH_SOURCE entsprechend anpassen. (Es wird im data Ordner innerhalb des src-Ordners gesucht, in dem auch die vorgefertigten Pfade liegen)


Befehle:
========
rosrun nonlinearcontroller simulator		-- Startet den Simulator

roslaunch nonlinearcontroller odom.launch	-- Startet Controller via Odometrie
			       amcl.launch	-- Startet Controller via AMCL

roslaunch custom_nodes odom.launch		-- Startet Pfadextrahierung via Odometrie
		        amcl.launch		-- Startet Pfadextrahierung via AMCL


----
Stefan Lindörfer/Julian Balling
03.10.2020
