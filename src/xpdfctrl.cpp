
/***************************************************************************
 *  xpdfctrl.cpp - ROS slide presenter
 *
 *  Created: Fri Sep  3 11:14:26 2010
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

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <stdexcept>

/** @class XpdfControl "xpdfctrl.h"
 * Xpdf Remote Control interface.
 * This class supports launching and controlling a remote Xpdf instance.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param remote remote name, this is Xpdf specific, read its documentation.
 * Basically it must be a simple system-wide unique string.
 */
XpdfControl::XpdfControl(const char *remote)
{
  __hidden = true;
  __remote = strdup(remote);
}

/** Destructor. */
XpdfControl::~XpdfControl()
{
  free(__remote);
}

/** Got to next page. */
void
XpdfControl::next_page()
{
  execute_command("nextPage");
}

/** Got to previous page. */
void
XpdfControl::prev_page()
{
  execute_command("prevPage");
}

/** Got to specific page.
 * @param n page number to go to
 */
void
XpdfControl::goto_page(unsigned int page)
{
  execute_command("'gotoPage(%u)'", page);
}

/** Hide Xpdf window. */
void
XpdfControl::hide()
{
  execute_command("quit");
}

/** Load PDF file.
 * This will start Xpdf if it was not running before and show the window if it
 * was hidden.
 * @param filename name of the file to open
 */
void
XpdfControl::load_file(const char *filename)
{
  execute_command("'' -fullscreen '%s'", filename);
}

/** Execute Xpdf command.
 * @param format a format string using standard sprintf notation followed by
 * the required number of arguments. The command is appended an ampersand, so
 * that the execution happens concurrently.
 */
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
