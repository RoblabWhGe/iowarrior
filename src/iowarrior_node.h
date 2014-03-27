/*
 * This file is part of the iowarrior package.
 *
 * Copyright (c)2014 by Robotics Lab 
 * in the Computer Science Department of the 
 * University of Applied Science Gelsenkirchen
 * 
 * Author: Benjamin König
 * Contributor: Stefan Wilkes
 *  
 * The package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <linux/types.h>
#include "iowarrior.h"
#include "iowarrior/iowarrior_setOutput.h"
#include "iowarrior/iowarrior_getStatus.h"

const unsigned int USB_DEVICE_ID_CODEMERCS_IOW40 = 0x1500;
const unsigned int USB_DEVICE_ID_CODEMERCS_IOW24 = 0x1501;
const unsigned int USB_DEVICE_ID_CODEMERCS_IOW56 = 0x1503;

/**
 * Ein Objekt dieser Klasse repraesentiert einen Knoten zur Ansteuerung einer
 * Relaiskarte.
 *
 * Der Knoten arbeitet auf Basis von Services, mittels denen es moeglich ist
 * die Pinne der einzelnen Ports anzusteuern und den aktuellen Status
 * abzufragen. Der Knoten kapselt die Ansteuerung des Treibers und faengt
 * Fehler der Hardware ab.
 *
 * @see
 * @author Benjamin Koenig
 */
class iowarrior_node {

public:

	/**
	 * Erzeugt einen Knoten zur Ansteuerung einer Relaiskarte.
	 *
         * @param device Der Pfad zum Warrior Device
	 * @param numberOfPorts Anzahl der verwendeten Ports
	 * @param portState Anfangszustand; eine Zeile setzt einen Port
	 */
    iowarrior_node(std::string &device, unsigned int numberOfPorts, unsigned int portState[]);

	/**
	 * Destructor
	 */
	~iowarrior_node();

	/**
	 * Hauptschleife des Knotens.
	 */
	void mainNodeLoop();

	/**
	 * Gibt die Informationen zur angeschlossenen Relaiskarte aus.
	 */
	void printDeviceInfo();

	/**
	 * Setzt den Zustand eines einzelnen Pins, eines entsprechenden Ports.
	 *
	 * @param port Index des Ports; ab 0
	 * @param pin Index des Pins; ab 0
	 * @param state true falls aktiviert werden soll
	 * @return
	 */
	bool setOutput(unsigned int port, unsigned int pin, bool state);

	/**
	 * Liest den aktuellen Zustand der Relaiskarte aus.
	 */
	void getStatus();

private:
	/**
	 * Informationen zum Gereat.
	 */
	struct iowarrior_info mInfo;

	/**
	 * Filehandle zur Initialisierung der Relaiskarte.
	 */
	int mFd;

	unsigned int mNumberOfPorts;

	/**
	 * Aktuelle Zustaende der Ports.
	 */
	unsigned int *mPortState;

	/**
	 * Zeitverzoegerung fuer das Timeout.
	 */
	struct timeval mTimeout;

	/**
	 * Node Handle.
	 */
	ros::NodeHandle mNodeHandle;

	/**
	 * Service Server zur Ansteuerung
	 */
	ros::ServiceServer mServServer_controller;

	/**
	 * Service Server zum Abfragen des Status
	 */
	ros::ServiceServer mServServer_status;

	/**
	 * A private node parameter
	 */
	// std::string mSomeParam;

	/**
	 * Lock zur Vermeidung der mehrfachen Statusabfrage.
	 */
	bool mStatusLock;

	/**
	 * Schreibt alle Ports und Pins.
	 *
	 * @param newPortState Ports (Zeilen) und Pins (Bits) die geschrieben werden.
	 * @return true falls erfolgreich.
	 */
	bool writeOutput(unsigned int newPortState[]);

	/**
	 * Callback-Funktion des Services zur Ansteuerung der Pins.
	 *
	 * @param req Eingangsnachricht
	 * @param res Ausgangsnachricht
	 * @return true falls erfolgreich
	 */
	bool setOutput_service(iowarrior::iowarrior_setOutput::Request &req,
			iowarrior::iowarrior_setOutput::Response &res);

	/**
	 * Callback-Funktion des Services zum Abfragen des Status.
	 *
	 * @param req Eingangsnachricht
	 * @param res Ausgangsnachricht
	 * @return true falls erfolgreich
	 */
	bool getStatus_service(iowarrior::iowarrior_getStatus::Request &req,
			iowarrior::iowarrior_getStatus::Response &res);

	/**
	 * Fuehrt alle notwendigen Aufgaben bei einem schwerwiegenden Fehler durch.
	 */
	void fatalError();
};
