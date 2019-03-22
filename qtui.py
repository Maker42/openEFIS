# Copyright (C) 2018  Garrett Herschleb
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

try:
    from PyQt5.QtGui import *
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *
except:
    from PyQt4.QtGui import *
    from PyQt4.QtCore import *

import os, time, glob
import logging

import pyavtools.fix as fix

import Globals

logger=logging.getLogger(__name__)

class FMSUI(QGraphicsView):
    def __init__(self, flight_plan_dir, parent=None):
        super(FMSUI, self).__init__(parent)
        time.sleep(.3)       # Wait for FMS to initialize
        self.parent = parent
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.focus_level = 0
        self.focus_index = 0
        self.focus_highlight = None
        self.flight_plan_dir = flight_plan_dir
        self.enc = fix.db.get_item("ENC1")
        self.selbtn = fix.db.get_item("BTN6")
        self.nwaypoints = 0

    def get_fltpln_options(self):
        return ["None"] + glob.glob(os.path.join (self.flight_plan_dir, '*.csv'))

    def resizeEvent(self, event):
        self.h = self.height()
        self.w = self.width()
        self.scene = QGraphicsScene(0,0, self.w*2, self.h)
        bg = self.scene.addRect (0,0, self.w*2, self.h, QPen(QColor(Qt.black)), QBrush(QColor(Qt.gray)))

        x = 0
        self.AltitudeDB = fix.db.get_item(Globals.SELECTED_ALTITUDE_KEY)
        self.alt = NumberWidget("Altitude", 3, 2, self.AltitudeDB, msd_limit=5)
        self.alt_item = self.scene.addWidget (self.alt)
        self.alt.move(x,0)
        x += self.alt.width()

        lpen = QPen(QColor(Qt.black))
        lpen.setWidth(5)
        self.scene.addLine (x,0, x,self.h, lpen)

        self.HeadDB = fix.db.get_item(Globals.SELECTED_HEADING_KEY)
        self.head = NumberWidget("Heading", 3, 0, self.HeadDB, msd_limit=3)
        self.head_item = self.scene.addWidget (self.head)
        self.head.move(x,0)
        x += self.head.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.VsDB = fix.db.get_item(Globals.SELECTED_CLIMB_RATE_KEY)
        self.vs = NumberWidget("Vert Speed", 3, 1, self.VsDB)
        self.vs_item = self.scene.addWidget (self.vs)
        self.vs.move(x,0)
        x += self.vs.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.AirspeedDB = fix.db.get_item(Globals.SELECTED_AIRSPEED_KEY)
        self.airspeed = NumberWidget("Air Speed", 3, 0, self.AirspeedDB)
        self.airspeed_item = self.scene.addWidget (self.airspeed)
        self.airspeed.move(x,0)
        x += self.airspeed.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.HnavModeDB = fix.db.get_item(Globals.HNAV_MODE_KEY)
        self.hnavmode = ChoiceWidget("HNAV Mode", ["Heading", "Flt Pln"], self.HnavModeDB)
        self.hnavmode_item = self.scene.addWidget(self.hnavmode)
        self.hnavmode.move (x, 0)
        x += self.hnavmode.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.VnavModeDB = fix.db.get_item(Globals.VNAV_MODE_KEY)
        self.vnavmode = ChoiceWidget("VNAV Mode", ["Vrt Spd", "Air Spd", "Fxd Pch", "Linear "], self.VnavModeDB)
        self.scene.addWidget(self.vnavmode)
        self.vnavmode_item = self.vnavmode.move (x, 0)
        x += self.vnavmode.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.AltSourceDB = fix.db.get_item(Globals.ALTITUDE_SOURCE_KEY)
        self.altsource = ChoiceWidget("ALT SRC", ["Selected", "Flt Pln"], self.AltSourceDB)
        self.altsource_item = self.scene.addWidget(self.altsource)
        self.altsource.move (x, 0)
        x += self.altsource.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.StartStrDB = fix.db.get_item(Globals.START_STRATEGY_KEY)
        self.startstr = ChoiceWidget("Start Strategy", ["First Waypoint", "Heading", "ASAP"], self.StartStrDB)
        self.startstr_item = self.scene.addWidget(self.startstr)
        self.startstr.move (x, 0)
        x += self.startstr.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.FixedPitchDB = fix.db.get_item(Globals.SELECTED_PITCH_KEY)
        self.fixedpitch = NumberWidget("Ptch", 2, 0, self.FixedPitchDB)
        self.fixedpitch_item = self.scene.addWidget(self.fixedpitch)
        self.fixedpitch.move (x, 0)
        x += self.fixedpitch.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.flight_plan = SelectMenuWidget("Flight Plan", self.get_fltpln_options(),
                                    self.new_flight_plan,
                                    SelectMenuWidget.MENU_ACTION_TYPE_FUNCTION,
                                    self.vnavmode.width()*2, self.height(), self.parent)
        self.flight_plan_item = self.scene.addWidget(self.flight_plan)
        self.flight_plan.move (x, 0)
        x += self.flight_plan.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.sel_waypoint = SelectMenuWidget("Sel Wp", [str (s) for s in list(range(self.nwaypoints))], 
                                    self.new_sel_waypoint,
                                    SelectMenuWidget.MENU_ACTION_TYPE_FUNCTION, 35,
                                    self.height(), self.parent)
        self.sel_waypoint_item = self.scene.addWidget(self.sel_waypoint)
        self.sel_waypoint.move (x, 0)
        x += self.sel_waypoint.width()
        self.scene.addLine (x,0, x,self.h, lpen)

        self.focus_targets = [self.alt, self.head, self.vs, self.airspeed, self.hnavmode, self.vnavmode,
                              self.altsource, self.startstr, self.fixedpitch, self.flight_plan,
                              self.sel_waypoint]

        self.setScene(self.scene)
        self.centerOn (self.alt_item)

    def new_flight_plan(self, i, choice):
        with open (choice, 'r') as flight_plan:
            wpnum = 0
            line_num = 0
            for waypoint_line in flight_plan:
                line_num += 1
                waypoint = waypoint_line.strip()
                if '#' in waypoint:
                    waypoint = waypoint[waypoint.index('#'):]
                    waypoint = waypoint.strip()
                if len(waypoint) == 0:
                    continue
                wpdata = waypoint.split(',')
                if len(wpdata) == 4:
                    wpid,wpalt,wplat,wplng = wpdata
                    wplat = float(wplat)
                    wplng = float(wplng)
                elif len(wpdata) == 6:
                    wpid,wpalt,wplat_deg,wplat_min,wplng_deg,wplng_min = wpdata
                    wplat_deg = float(wplat_deg)
                    lsign = 1.0 if wplat_deg >= 0 else -1.0
                    wplat = wplat_deg + lsign*float(wplat_min)/60.0
                    wplng_deg = float(wplng_deg)
                    lsign = 1.0 if wplng_deg >= 0 else -1.0
                    wplng = wplng_deg + lsign*float(wplng_min)/60.0
                elif len(wpdata) == 8:
                    wpid,wpalt,wplat_deg,wplat_min,wplat_sec,wplng_deg,wplng_min,wplng_sec = wpdata
                    wplat_deg = float(wplat_deg)
                    lsign = 1.0 if wplat_deg >= 0 else -1.0
                    wplat = wplat_deg + lsign*float(wplat_min)/60.0 + lsign*float(wplat_sec)/3600.0
                    wplng_deg = float(wplng_deg)
                    lsign = 1.0 if wplng_deg >= 0 else -1.0
                    wplng = wplng_deg + lsign*float(wplng_min)/60.0 + lsign*float(wplng_sec)/3600.0
                else:
                    logger.error ("Unrecognized flight plan line in %s, line %d: %s",
                            choice, line_num, waypoint_line)
                    break
                fix.db.get_item(Globals.WAYPOINT_ID_KEY  + str(wpnum)).value = wpid
                fix.db.get_item(Globals.WAYPOINT_LAT_KEY + str(wpnum)).value = wplat
                fix.db.get_item(Globals.WAYPOINT_LNG_KEY + str(wpnum)).value = wplng
                fix.db.get_item(Globals.WAYPOINT_ALT_KEY + str(wpnum)).value = float(wpalt)
                wpnum += 1
            fix.db.get_item(Globals.WAYPOINT_ID_KEY  + str(wpnum)).value = ''
            self.nwaypoints = wpnum
            self.sel_waypoint.setChoices([str (s) for s in list(range(self.nwaypoints))])
            flight_plan.close()

    def new_sel_waypoint(self, i, _):
        fix.db.get_item(Globals.SELECTED_WAYPOINT_KEY).value = i
    
    def focus(self):
        self.flight_plan.setChoices(self.get_fltpln_options())
        self.focus_level = 0
        self.focus_index = 0
        self.last_rotary = self.enc.value
        self.enc.valueChanged[int].connect(self.rotary)
        self.selbtn.valueChanged[bool].connect(self.select)
        self.draw_focus (True)

    def defocus(self):
        self.draw_focus (False)
        self.enc.valueChanged[int].disconnect(self.rotary)
        self.selbtn.valueChanged[bool].disconnect(self.select)

    def draw_focus(self, yes):
        if self.focus_highlight is not None:
            self.scene.removeItem (self.focus_highlight)
            self.focus_highlight = None
        if yes:
            param = self.focus_targets[self.focus_index]
            if self.focus_level == 0:
                # Draw a highlighted box around the current parameter / focus target
                rect = QRectF(param.frameGeometry())
            else:
                rect = QRectF(param.draw_focus (yes))
                rect.moveLeft (rect.x() + param.x())
            hpen = QPen(QColor(Qt.green))
            hpen.setWidth(3)
            hbrush = QBrush()
            self.focus_highlight = self.scene.addRect (rect, hpen, hbrush)
            self.centerOn (self.focus_highlight)

    def rotary(self, event):
        diff = self.enc.value - self.last_rotary 
        if self.focus_level == 0:
            self.focus_index += diff
            if self.focus_index < 0:
                self.focus_index = 0
            elif self.focus_index >= len(self.focus_targets):
                self.focus_index = len(self.focus_targets)-1
        else:
            param = self.focus_targets[self.focus_index]
            param.changeby(diff)
        self.draw_focus(True)
        self.last_rotary = self.enc.value

    def select(self, event):
        if self.selbtn.value:
            param = self.focus_targets[self.focus_index]
            if self.focus_level == 0:
                self.focus_level += 1
                param.focus_first()
            else:
                if not param.focus_next():
                    self.focus_level = 0
            self.draw_focus(True)

    def select_flight_plan(self, i, fn):
        path = os.path.append(self.flight_plan_dir, fn)

class NumberWidget(QWidget):
    def __init__(self, title, num_var_digits, num_zero_digits, dbitem, parent=None, msd_limit=9):
        super(NumberWidget, self).__init__(parent)
        self.parent = parent
        self.dbitem = dbitem
        self.focus_index = 0
        self.var_digits = list()
        self.dials = list()
        font = QFont()
        font.setBold(True)
        var_digit_spacing = 15
        self.w = (var_digit_spacing*num_var_digits) + (10*num_zero_digits) + 10
        self.h = 50
        self.layout = QGridLayout()
        self.layout.setSpacing(0)
        self.lock = False
        msd=True
        for i in range(num_var_digits):
            dial = QDial(self)
            dial.setMinimum(0)
            if msd:
                dial.setMaximum(msd_limit)
                msd = False
            else:
                dial.setMaximum(9)
            dial.setWrapping(True)
            dial.setFixedSize(15,15)
            dial.valueChanged.connect(self.dialChanged)
            self.dials.append(dial)
            self.layout.addWidget(dial, 2, i)
            digit = QLabel("0", parent=self)
            self.var_digits.append (digit)
            digit.setFont(font)
            self.layout.addWidget(digit, 1, i, Qt.AlignCenter)
            last_i = i

        self.num_zero_digits = num_zero_digits
        if num_zero_digits > 0:
            self.zero_digit = QLabel("0" * num_zero_digits)
            self.zero_digit.setFont(font)
            self.layout.addWidget(self.zero_digit, 1, last_i+1)
        else:
            self.zero_digit = None
        tlabel = QLabel(title)
        self.layout.addWidget(tlabel, 0, 0, 1, -1)
        self.setValue(self.dbitem.value)
        self.setLayout(self.layout)
        self.adjustSize()

    def setValue(self, v):
        self._value = v
        self.lock = True
        divisor = int(10**(len(self.var_digits) + self.num_zero_digits - 1))
        for dial,digit in zip(self.dials,self.var_digits):
            val = int(self._value / divisor) % 10
            t = str(val)
            dial.setValue(val)
            digit.setText(t)
            divisor /= 10
        self.lock = False

    def dialChanged(self, event):
        if not self.lock:
            multiplier = int(10**(len(self.var_digits) + self.num_zero_digits - 1))
            result = 0
            for dial,digit in zip(self.dials,self.var_digits):
                val = dial.value()
                result += multiplier * val
                digit.setText(str(val))
                multiplier /= 10
            self._value = result
            self.dbitem.value = self._value

    def draw_focus (self, yes):
        dial = self.dials[self.focus_index]
        digit = self.var_digits[self.focus_index]
        dial.setFocus()
        ret = dial.frameGeometry()
        ret |= digit.frameGeometry()
        return ret

    def changeby(self, diff):
        d = self.dials[self.focus_index]
        d.setValue((d.value() + diff) % 10)
    
    def focus_first(self):
        self.focus_index = 0
    
    def focus_next(self):
        if self.focus_index + 1 >= len(self.dials):
            return False
        else:
            self.focus_index += 1
            return True

class ChoiceWidget(QWidget):
    def __init__(self, title, choices, dbitem, parent=None):
        super(ChoiceWidget, self).__init__(parent)
        self.parent = parent
        self.dbitem = dbitem
        font = QFont()
        font.setPixelSize(10)
        self.setFont(font)
        self.layout = QGridLayout()
        self.layout.setSpacing(0)
        self.bg = QButtonGroup()
        self.buttons = list()
        row = 1
        col = 0

        for i,choice in enumerate(choices):
            button = QRadioButton(choice)
            if i == 0:
                button.setChecked(True)
            self.bg.addButton (button, i)
            self.layout.addWidget(button, row, col)
            self.buttons.append (button)
            row += 1
            if row >= 3:
                row = 1
                col += 1
        self.bg.buttonClicked[int].connect(self.selChanged)

        tlabel = QLabel(title)
        lfont = QFont()
        lfont.setPixelSize(10)
        lfont.setBold(True)
        tlabel.setFont(lfont)
        self.layout.addWidget(tlabel, 0, 0, 1, -1)
        self.setLayout(self.layout)
        self.adjustSize()

    def selChanged(self, event):
        #print ("new sel = %d"%self.bg.checkedId())
        self.dbitem.value = self.bg.checkedId()

    def draw_focus (self, yes):
        ret = QRect()
        for b in self.buttons:
            ret |= b.frameGeometry()
        return ret

    def changeby(self, diff):
        newid = self.bg.checkedId()
        newid += diff
        if newid < 0:
            newid = 0
        elif newid >= len(self.buttons):
            newid = len(self.buttons) - 1
        self.buttons[newid].setChecked(True)
        #print ("new rotary sel %s = %d"%(self.dbitem.key, self.bg.checkedId()))
        self.dbitem.value = self.bg.checkedId()
    
    def focus_first(self):
        pass
    
    def focus_next(self):
        return False

class SelectMenuWidget(QGraphicsView):
    MENU_ACTION_TYPE_DBSEL_INDEX=0
    MENU_ACTION_TYPE_DBSEL_TEXT=1
    MENU_ACTION_TYPE_FUNCTION=2
    LEFT_MARGIN=5
    def __init__(self, title, choices, action_item, action_type, width, height, parent=None):
        super(SelectMenuWidget, self).__init__()
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.parent = parent
        self.action_item = action_item
        self.action_type = action_type
        self.title = title
        self.choices = choices
        self.menu = None
        self.current_index = 0
        font = QFont()
        font.setPixelSize(10)
        self.setFont(font)
        self.resize(width, height)

    def resizeEvent(self, event):
        self.h = self.height()
        self.w = self.width()
        t = QGraphicsSimpleTextItem ("9")
        t.setFont(self.font())
        font_height = t.boundingRect().height()
        self.scene = QGraphicsScene(0,0, self.w, font_height * (len(self.choices)+1) + self.h)
        bgbrush = QBrush(QColor("#d0d0d0"))
        self.scene.addRect (0,0, self.scene.width(), self.scene.height(), QPen(QColor(Qt.black)), bgbrush)
        self.setScene(self.scene)
        hpen = QPen()
        hbrush = QBrush(QColor(Qt.white))
        rect = QRectF (0,self.h/2, self.w, font_height)
        self.focus_highlight = self.scene.addRect (rect, hpen, hbrush)
        y = self.h/2
        self.actions = list()

        for c in self.choices:
            self.actions.append (self.scene.addSimpleText (c))
            self.actions[-1].setY(y)
            self.actions[-1].setX(self.LEFT_MARGIN)
            y += font_height

        rect = QRectF (0,font_height/2, self.w, font_height)
        self.title_label_rect = self.scene.addRect (rect, hpen, bgbrush)
        self.title_label = self.scene.addSimpleText (self.title)
        self.title_label.setY(font_height/2)
        self.title_label.setX(self.LEFT_MARGIN)

        self.centerOn (self.focus_highlight)

    def setChoices(self, c):
        self.choices = c
        self.current_index = 0
        self.resizeEvent(0)

    def changeby(self, diff):
        newid = self.current_index
        newid += diff
        if newid < 0:
            newid = 0
        elif newid >= len(self.choices):
            newid = len(self.choices)-1
        self.current_index = newid
        h = self.actions[0].boundingRect().height()
        y = self.actions[self.current_index].y()
        self.focus_highlight.setRect (QRectF (0, y, self.w, h))
        self.title_label_rect.setRect (QRectF (0, y-self.h/2+h/2, self.w, h))
        self.title_label.setY(y-self.h/2+h/2)
        self.centerOn (self.focus_highlight)
    
    def menu_action(self):
        if self.action_type == self.MENU_ACTION_TYPE_DBSEL_INDEX:
            fix.db.get_item(self.action_item).value = self.current_index
        elif self.action_type == self.MENU_ACTION_TYPE_DBSEL_TEXT:
            fix.db.get_item(self.action_item).value = self.choices[self.current_index]
        elif self.action_type == self.MENU_ACTION_TYPE_FUNCTION:
            self.action_item (self.current_index, self.choices[self.current_index])

    def draw_focus (self, yes):
        return QRect(0,0,self.w, self.h)

    def focus_first(self):
        pass
    
    def focus_next(self):
        self.menu_action()
        return False

