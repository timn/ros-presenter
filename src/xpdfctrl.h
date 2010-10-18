
/***************************************************************************
 *  xpdfctrl.h - ROS slide presenter
 *
 *  Created: Fri Sep  3 11:12:12 2010
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

class XpdfControl
{
 public:
  XpdfControl(const char *remote);
  ~XpdfControl();

  void next_page();
  void prev_page();
  void goto_page(unsigned int page);
  void load_file(const char *filename);
  void hide();

 private:
  void execute_command(const char *format, ...);

 private:
  bool __hidden;
  char *__remote;
};
