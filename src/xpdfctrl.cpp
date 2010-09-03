
/***************************************************************************
 *  xpdfctrl.cpp - ROS slide presenter
 *
 *  Created: Fri Sep  3 11:14:26 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
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

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <stdexcept>

XpdfControl::XpdfControl(const char *remote)
{
  __hidden = true;
  __remote = strdup(remote);
}

XpdfControl::~XpdfControl()
{
  free(__remote);
}


void
XpdfControl::next_page()
{
  execute_command("nextPage");
}

void
XpdfControl::prev_page()
{
  execute_command("prevPage");
}

void
XpdfControl::goto_page(unsigned int page)
{
  execute_command("'gotoPage(%u)'", page);
}

void
XpdfControl::hide()
{
  execute_command("quit");
}

void
XpdfControl::load_file(const char *filename)
{
  execute_command("'' -fullscreen '%s'", filename);
}


void
XpdfControl::execute_command(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char *cmd;
  if (vasprintf(&cmd, format, arg) != -1) {
    std::string cmds = cmd; free(cmd);
    char *cmdline; 
    if (asprintf(&cmdline, "xpdf -remote '%s' -exec %s &", __remote, cmds.c_str()) != -1) {
      std::string cmdlines = cmdline; free(cmdline);
      if (system(cmdlines.c_str()) != 0) {
	std::runtime_error e(std::string("Executing ") + cmdlines + "failed");
	throw e;
      }
    } else {
      throw std::runtime_error("Allocating memory for command string failed");
    }
  } else {
    throw std::runtime_error("Allocating memory for Xpdf command string failed");
  }
  va_end(arg);
}
