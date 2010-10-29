
/***************************************************************************
 *  presenter.cpp - ROS slide presenter
 *
 *  Created: Fri Sep  3 10:39:50 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Carnegie Mellon University
 *             2010  Intel Labs Pittsburgh, Intel Research
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "xpdfctrl.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <presenter/GotoPage.h>
#include <presenter/Load.h>


/** Interface XpdfControl with ROS.
 * @author Tim Niemueller
 */
class Presenter
{
 public:
  /** Constructor. */
  Presenter()
    : __xpdfctrl("ROS_XPDF")
  {
    std::string name = ros::this_node::getName();
    __srv_load = __node_handle.advertiseService(name + "/load", &Presenter::load, this);
    __srv_next_page = __node_handle.advertiseService(name + "/page/next", &Presenter::next_page, this);
    __srv_prev_page = __node_handle.advertiseService(name + "/page/prev", &Presenter::prev_page, this);
    __srv_goto_page = __node_handle.advertiseService(name + "/page/goto", &Presenter::goto_page, this);
    __srv_hide = __node_handle.advertiseService(name + "/hide", &Presenter::hide, this);
  }

  // Service callbacks

  bool next_page(std_srvs::Empty::Request &req,
		 std_srvs::Empty::Response &resp)
  {
    __xpdfctrl.next_page();
    return true;
  }

  bool prev_page(std_srvs::Empty::Request &req,
		 std_srvs::Empty::Response &resp)
  {
    __xpdfctrl.prev_page();
    return true;
  }

  bool hide(std_srvs::Empty::Request &req,
	    std_srvs::Empty::Response &resp)
  {
    __xpdfctrl.hide();
    return true;
  }

  bool goto_page(presenter::GotoPage::Request &req,
		 presenter::GotoPage::Response &resp)
  {
    __xpdfctrl.goto_page(req.page);
    return true;
  }

  bool load(presenter::Load::Request &req,
	    presenter::Load::Response &resp)
  {
    // check file name
    std::string filename = req.filename;
    __xpdfctrl.load_file(filename.c_str());
    return true;
  }

 private:
  ros::NodeHandle __node_handle;
  ros::ServiceServer __srv_load;
  ros::ServiceServer __srv_next_page;
  ros::ServiceServer __srv_prev_page;
  ros::ServiceServer __srv_hide;
  ros::ServiceServer __srv_goto_page;

  XpdfControl __xpdfctrl;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "presenter");
  Presenter presenter;
  ros::spin();
  return 0;
}
