/**
 * Copyright (c) 2011, University of Applied Sciences Gelsenkirchen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Washington University in St. Louis nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Benjamin Koenig
 * Contributor: Stefan Wilkes
 * =============================================================================
 *
 *  iowarrior_node.h
 *
 *  Provides services to control the relays.
 *
 * =============================================================================
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
