/*
 * start_gui.cpp
 *
 *  Created on: 15.05.2017
 *      Author: basti
 */

#include <QApplication>
#include <ros/ros.h>
#include "DisplayGUI.h"
#include <QDebug>



int main (int argc, char** argv){
	ros::init(argc, argv, "start_gui");
	QApplication app(argc, argv);
//	qDebug() << " constructor";
	Display_GUI w;// = new DisplayGUI();
//	qDebug() << " show";
	w.show();
//	qDebug() << " exec";

	return app.exec();
}


