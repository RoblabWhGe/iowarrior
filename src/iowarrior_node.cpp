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
#include "iowarrior_node.h"

iowarrior_node::iowarrior_node(std::string &device, unsigned int numberOfPorts,
		unsigned int portState[]) {

	mFd = -1;
	/* we try to open the device attached to /dev/usb/iowarrior0 */
	if ((mFd = open(device.c_str(), O_RDWR)) < 0) {
        ROS_ERROR("IOWarrior (%s) open failed", device.c_str());
		fatalError();
	}
	/* lets see what product we have */
	if (ioctl(mFd, IOW_GETINFO, &mInfo)) {
		ROS_ERROR("Unable to retrieve device info");
		fatalError();
	}

	mServServer_controller = mNodeHandle.advertiseService("iowarrior/control",
			&iowarrior_node::setOutput_service, this);
	mServServer_status = mNodeHandle.advertiseService("iowarrior/status",
			&iowarrior_node::getStatus_service, this);
	mNumberOfPorts = numberOfPorts;

	if (numberOfPorts > mInfo.report_size) {
		ROS_ERROR("Die Anzahl der nutzbaren Ports ist groesser als die Anzahl der "
				"verfuegbaren! \n");
		fatalError();
	} else if (numberOfPorts < mInfo.report_size) {
		ROS_DEBUG("Die Schnittstelle bietet mehr Ports als genutzt werden. \n");
	}

	mPortState = new unsigned int[mInfo.report_size];
	if (mPortState == NULL) {
		ROS_INFO("Speicherfehler");
		fatalError();
	}
	for (unsigned int i = 0; i < mInfo.report_size; i++) {
		if (i < numberOfPorts) {
			mPortState[i] = portState[i];
		} else {
			mPortState[i] = 0xFF;
		}
	}

	mTimeout.tv_sec = 10;
	mTimeout.tv_usec = 0;

	if (!writeOutput(portState)) {
		ROS_ERROR("Fehler bei der Initialisierung");
		fatalError();
	}
}

void iowarrior_node::fatalError() {
	close(mFd);
	exit(0);
}

void iowarrior_node::printDeviceInfo() {
	ROS_INFO("The device attached to /dev/usb/iowarrior0\n");
	ROS_INFO("VendorId=%04x\n", mInfo.vendor);
	ROS_INFO("ProductId=%04x ", mInfo.product);
	if (mInfo.product == USB_DEVICE_ID_CODEMERCS_IOW40)
		ROS_INFO("(IOWarrior40)\n");
	else if (mInfo.product == USB_DEVICE_ID_CODEMERCS_IOW24)
		ROS_INFO("(IOWarrior24)\n");
	else if (mInfo.product == USB_DEVICE_ID_CODEMERCS_IOW56) {
		ROS_INFO("(IOWarrior56)\n");
	} else
		ROS_INFO("(Ooops, unknown device!)\n");
	ROS_INFO("Serial=0x%s\nRevision=0x%04x\n", mInfo.serial, mInfo.revision);
	ROS_INFO("Speed=");
	if (mInfo.speed == 1)
		ROS_INFO("Low Speed (USB 1.1)\n");
	else if (mInfo.speed == 2)
		ROS_INFO("Full Speed (USB 1.1)\n");
	else if (mInfo.speed == 3)
		ROS_INFO("High Speed (USB 2.0)\n");
	else
		ROS_INFO("Speed is unknown!\n");
	ROS_INFO("Power=%dmA\nPacketSize=%d\n", mInfo.power, mInfo.report_size);
	/* checking for the IO-PIN interface */
	if (mInfo.if_num != 0) {
		ROS_INFO("This is not the IO-PIN interface\n");
		close(mFd);
		exit(0);
	}
}

bool iowarrior_node::setOutput_service(
		iowarrior::iowarrior_setOutput::Request &req,
		iowarrior::iowarrior_setOutput::Response &res) {
	bool success = setOutput(req.port, req.pin, req.state);
	res.success = success;
	return success;
}

bool iowarrior_node::setOutput(unsigned int port, unsigned int pin, bool state) {
	/* Falls der Pin gesetzt werden soll, jedoch schon gesetzt war oder
	 * der Pin zurueckgesetzt werden soll und nicht gesetzt ist, muss nichts
	 * durchgefuehrt werden.
	 */
	if ((state && (mPortState[port] & (1 << pin)) > 0) || (!state
			&& (~mPortState[port] & (1 << pin)) > 0)) {
		ROS_DEBUG("Schreiben nicht notwendig da Pin schon den Zustand hat.");
		return true;
	}

	// Erzeugung eines neuen Statusspeichers mit Kopie des alten Status.
	unsigned int* newPortState = new unsigned int[mInfo.report_size];
	if (newPortState == NULL) {
		ROS_INFO("Speicherfehler");
		fatalError();
	}
	for (unsigned int i = 0; i < mInfo.report_size; i++) {
		if (i < mNumberOfPorts) {
			newPortState[i] = mPortState[i];
		} else {
			newPortState[i] = 0xFF;
		}
	}
	// Pin setzen bzw. zuruecksetzen
	if (state) {
		newPortState[port] = newPortState[port] | (1 << pin);
	} else {
		newPortState[port] = newPortState[port] & ~(1 << pin);
	}
	// Alle Zustaende schreiben
	bool success = writeOutput(newPortState);
	/* Sofern erfolgreich geschrieben werden konnte, werden die neuen Zustaende
	 * zu den aktuellen Zustaenden.
	 */
	if (success && mPortState != NULL) {
		delete[] mPortState;
		mPortState = newPortState;
	}
	return success;
}

bool iowarrior_node::writeOutput(unsigned int newPortState[]) {
	fd_set wrfds;
	FD_ZERO(&wrfds);
	FD_SET(mFd, &wrfds);
	int result = select(mFd + 1, NULL, &wrfds, NULL, &mTimeout);
	if (result == -1) {
		ROS_ERROR("Fehler beim Select.");
	} else if (result == 0) {
		ROS_DEBUG("Timeout beim Schreiben.");
	} else {
		if (FD_ISSET(mFd,&wrfds)) {
			result = write(mFd, newPortState, mInfo.report_size);
			if ((unsigned int) result != mInfo.report_size) {
				int errcode = errno;
				if (errcode == ENODEV) {
					ROS_ERROR("Kabel beim Schreiben abgezogen.");
				} else {
					ROS_ERROR("Fehler beim Schreiben.");
				}
			} else {
				ROS_DEBUG("Schreiben erfolgreich.");
				mStatusLock = true;
				// ToDo: Workaround, besser Zeiten vergleichen damit nicht direkt nach dem Schreiben getStatus gemacht wird
				return true;
			}
		}
	}
	return false;
}

bool iowarrior_node::getStatus_service(iowarrior::iowarrior_getStatus::Request &req,
		iowarrior::iowarrior_getStatus::Response &res) {
	getStatus();
	res.state = ((mPortState[req.port] & (1 << req.pin)) > 0);
	return true;
}

void iowarrior_node::getStatus() {
	/* Sofern das Lock nicht gesetzt ist, wird der aktuelle Zustand vom Gereat
	 * abgefragt.
	 */
	if (!mStatusLock) {
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(mFd, &rfds);
		int result = select(mFd + 1, &rfds, NULL, NULL, &mTimeout);

		if (result == -1) {
			ROS_DEBUG("Fehler beim Select.");
			fatalError();
		} else if (result == 0) {
			ROS_DEBUG("Timeout beim Lesen.");
		} else {
			unsigned int* in_buffer = new unsigned int[mInfo.report_size];
			if (in_buffer == NULL) {
				ROS_INFO("Speicherfehler");
				fatalError();
			}
			if (FD_ISSET(mFd,&rfds)) {
				memset(&in_buffer, 0, sizeof(in_buffer));
				result = read(mFd, in_buffer, mInfo.report_size);
				if ((unsigned int) result != mInfo.report_size) {
					int errcode = errno;
					if (errcode == ENODEV)
						ROS_DEBUG("Kabel beim Lesen gezogen.");
					else
						ROS_DEBUG("Fehler beim Lesen.");
				} else {
					delete[] mPortState;
					mPortState = in_buffer;
					mStatusLock = true;
				}
			}
		}
	}
	/* Ausgabe der aktuellen Portzustaende.
	 */
	for (unsigned int i = 0; i < mNumberOfPorts; i++) {
		ROS_DEBUG("Port%d=%02x", i, mPortState[i]);
	}
}

iowarrior_node::~iowarrior_node() {
	unsigned int newPortState[mNumberOfPorts];
	for (unsigned int i = 0; i < mNumberOfPorts; i++) {
		newPortState[i] = 0xFF;
	}

	if (!writeOutput(newPortState)) {
		ROS_ERROR("Fehler beim Beenden");
		fatalError();
	}
	delete mPortState;
	close(mFd);
}

void iowarrior_node::mainNodeLoop() {
	ros::spin();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "iowarrior_node");

	ros::NodeHandle nh("~");
	int initValue;
    std::string device;
	nh.param("InitValue", initValue, 0x3);
    nh.param<std::string>("device", device, "/dev/usb/iowarrior0");

	unsigned int newPortState[] = { initValue };
	iowarrior_node iwn(device, 1, newPortState);
	iwn.mainNodeLoop();
	return 0;
}
