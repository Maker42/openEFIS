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

import math, time

import tkinter as tk

RAD_DEG=math.pi / 180
DEG_RAD=180 / math.pi

class TkDisplay(tk.Frame):
    def __init__(self, width, height, master=None):
        super().__init__(master)
        self.width = width
        self.height = height
        self.pixels_pitch = height/90
        self.pack()
        self.attitude_items = list()
        self.instruments_items = list()
        self.create_widgets()

    def create_widgets(self):

        self.attitude_width = int(round(self.width * 0.95))
        self.attitude_height = int(round(self.height * 0.60))
        self.attitude = tk.Canvas(self, width=self.attitude_width, height=self.attitude_height)
        self.attitude.pack(side="top")

        self.instruments_width = int(round(self.width * 0.95))
        self.instruments_height = int(round(self.height * 0.60))
        self.instruments = tk.Canvas(self, width=self.instruments_width, height=self.instruments_height)
        self.instruments.pack(side="top")

    def display_horizon(self, pitch, roll):
        for i in self.attitude_items:
            self.attitude.delete (i)
        middle_height = self.attitude_height / 2 - pitch * self.pixels_pitch
        side_offset = math.sin(roll * RAD_DEG) * self.attitude_width / 2
        self.attitude_items = list()
        p = self.attitude.create_polygon (0,middle_height-side_offset,
                self.attitude_width,middle_height+side_offset,
                self.attitude_width,self.attitude_height,
                0,self.attitude_height,
                fill="green")
        self.attitude_items.append(p)
        p = self.attitude.create_polygon (0,0,
                self.attitude_width,0,
                self.attitude_width,middle_height+side_offset,
                0,middle_height-side_offset,
                fill="blue")
        self.attitude_items.append(p)
        self.display_pitchlines()

    def display_pitchlines(self):
        reflinewidth_2 = (self.attitude_width / 4) / 2
        middle_height = self.attitude_height / 2
        middle_width = self.attitude_width / 2
        for pitch_ref in range (-15,16,5):
            y = middle_height - pitch_ref * self.pixels_pitch
            fill = "yellow"
            linewidth=2
            width_add = 0
            if pitch_ref == 0:
                width_add = (self.attitude_width / 2) * .5
                fill = "white"
                linewidth=3
            elif pitch_ref % 10 == 0:
                width_add = 20
            xoff = reflinewidth_2 + width_add
            pr = self.attitude.create_line (
                    middle_width-xoff, y,
                    middle_width+xoff, y,
                    fill=fill, width=linewidth)
            self.attitude_items.append(pr)

    def display_airspeed(self, airspeed):
        tapewidth = 80
        taperight = self.attitude_width - 5
        tapeleft = taperight - tapewidth
        tapetop = 5
        tapebottom = self.attitude_height - 5

        taperect = self.attitude.create_rectangle (
                taperight, tapetop,
                tapeleft, tapebottom,
                fill="grey")
        self.attitude_items.append(taperect)
        linex = taperight - tapewidth/2
        tapeline = self.attitude.create_line (linex,tapetop,
                linex,tapebottom, width=2, fill="white")
        self.attitude_items.append(tapeline)

        pixels_knot = 3
        middle_height = self.attitude_height / 2
        y0knots = middle_height + airspeed * pixels_knot
        yknots = y0knots
        knots = 0
        while yknots > tapetop-2:
            if yknots < tapebottom - 5:
                ticksize = 5
                if (knots % 10 == 0):
                    ticksize += 2
                tl = self.attitude.create_line (linex,yknots,
                        linex+ticksize,yknots, fill="white")
                self.attitude_items.append(tl)

                if (knots % 10 == 0):
                    label = self.attitude.create_text (linex+10,yknots,
                            text=str(knots), anchor=tk.W)
                    self.attitude_items.append(label)
            knots += 5
            yknots -= pixels_knot*5
        refleft = tapeleft+2
        refright = linex
        refheight_2 = 10
        reftop = middle_height - refheight_2
        refbottom = middle_height + refheight_2
        reftaperx = refright - 5
        ref = self.attitude.create_polygon (refleft,reftop,
                reftaperx,reftop,
                refright,middle_height,
                reftaperx,refbottom,
                refleft,refbottom,
                fill="yellow")
        self.attitude_items.append(ref)
        curas = self.attitude.create_text (reftaperx-3, middle_height,
                text=str(airspeed), fill="black", anchor=tk.E)
        self.attitude_items.append(curas)

    def display_altitude(self, altitude):
        tapewidth = 80
        tapeleft = 5
        taperight = tapeleft + tapewidth
        tapetop = 5
        tapebottom = self.attitude_height - 5

        taperect = self.attitude.create_rectangle (
                taperight, tapetop,
                tapeleft, tapebottom,
                fill="grey")
        self.attitude_items.append(taperect)
        linex = taperight - tapewidth/2
        tapeline = self.attitude.create_line (linex,tapetop,
                linex,tapebottom, width=2, fill="white")
        self.attitude_items.append(tapeline)

        pixels_foot = .2
        middle_height = self.attitude_height / 2
        y0feet = middle_height + altitude * pixels_foot
        yfeet = y0feet
        feet = 0
        while yfeet > tapetop-2:
            if yfeet < tapebottom - 5:
                ticksize = 5
                if (feet % 100 == 0):
                    ticksize += 2
                tl = self.attitude.create_line (linex,yfeet,
                        linex-ticksize,yfeet, fill="white")
                self.attitude_items.append(tl)

                if (feet % 100 == 0):
                    label = self.attitude.create_text (linex-10,yfeet,
                            text=str(feet), anchor=tk.E)
                    self.attitude_items.append(label)
            feet += 50
            yfeet -= pixels_foot*50
        refright = taperight-2
        refleft = linex
        refheight_2 = 10
        reftop = middle_height - refheight_2
        refbottom = middle_height + refheight_2
        reftaperx = refleft + 5
        ref = self.attitude.create_polygon (refright,reftop,
                reftaperx,reftop,
                refleft,middle_height,
                reftaperx,refbottom,
                refright,refbottom,
                fill="yellow")
        self.attitude_items.append(ref)
        curas = self.attitude.create_text (reftaperx-3, middle_height,
                text=str(altitude), fill="black", anchor=tk.W)
        self.attitude_items.append(curas)

    def display_climb_rate(self, climb_rate):   # In feet per minute
        centerx = 65
        middle_height = self.attitude_height / 2
        pixels_per_fpm = .05
        if climb_rate >= -100:
            beginy = middle_height - 20
            endy = beginy - climb_rate * pixels_per_fpm
            if endy < 60:
                endy = 60
            anc = tk.S
        else:
            beginy = middle_height + 20
            endy = beginy - climb_rate * pixels_per_fpm
            if endy > self.attitude_height-60:
                endy = self.attitude_height-60
            anc = tk.N
        if abs(climb_rate) > 250:
            l = self.attitude.create_line (centerx,beginy,
                centerx,endy,
                arrow=tk.LAST, width=3)
            self.attitude_items.append(l)
        t = self.attitude.create_text (centerx,endy,
                text=str(climb_rate) + "\nfpm", anchor=anc)
        self.attitude_items.append(t)

    def display_instruments(self, heading, cdi, turn_rate, yaw):
        for i in self.instruments_items:
            self.instruments.delete (i)
        middle_height = self.instruments_height / 2
        middle_width = self.instruments_width / 2
        hsi_size_2 = middle_height * 3 / 6
        self.instruments_items = list()
        hsicircle = self.instruments.create_oval(middle_width-hsi_size_2, middle_height-hsi_size_2,
                middle_width+hsi_size_2, middle_height+hsi_size_2, width=2)
        self.instruments_items.append(hsicircle)
        hsi_angle = -heading
        if hsi_angle < 0:
            hsi_angle += 360
        for draw_heading in range(0,360,5):
            tickx = math.sin(hsi_angle * RAD_DEG) * hsi_size_2
            ticky = -math.cos(hsi_angle * RAD_DEG) * hsi_size_2
            tickx += middle_width
            ticky += middle_height
            ticksize = 8
            if draw_heading % 10 == 0:
                ticksize += 4
            itickx = math.sin(hsi_angle * RAD_DEG) * (hsi_size_2 - ticksize)
            iticky = -math.cos(hsi_angle * RAD_DEG) * (hsi_size_2 - ticksize)
            itickx += middle_width
            iticky += middle_height
            tick = self.instruments.create_line (tickx,ticky,
                    itickx,iticky, width=2)
            self.instruments_items.append(tick)
            if draw_heading % 30 == 0:
                if hsi_angle < 23:
                    anc = tk.S
                elif hsi_angle < 68:
                    anc = tk.SW
                elif hsi_angle < 113:
                    anc = tk.W
                elif hsi_angle < 158:
                    anc = tk.NW
                elif hsi_angle < 203:
                    anc = tk.N
                elif hsi_angle < 248:
                    anc = tk.NE
                elif hsi_angle < 293:
                    anc = tk.E
                elif hsi_angle < 360-23:
                    anc = tk.SE
                else:
                    anc = tk.S
                ltickx = math.sin(hsi_angle * RAD_DEG) * (hsi_size_2 + 2)
                lticky = -math.cos(hsi_angle * RAD_DEG) * (hsi_size_2 + 2)
                ltickx += middle_width
                lticky += middle_height + 2
                label = self.instruments.create_text (ltickx, lticky,
                        text=str(draw_heading), anchor=anc)
                self.instruments_items.append(label)
            hsi_angle += 5
            if hsi_angle >= 360:
                hsi_angle -= 360

        # Now draw the CDI fixed part
        fixed_top = middle_height - hsi_size_2 + 13
        move_top = fixed_top + 20
        fixed_bottom = middle_height + hsi_size_2 - 13
        move_bottom = fixed_bottom - 20
        l = self.instruments.create_rectangle (middle_width-2,fixed_top,
                middle_width+2, move_top, outline="black", fill="yellow")
        self.instruments_items.append(l)
        l = self.instruments.create_rectangle (middle_width-2,fixed_bottom,
                middle_width+2, move_bottom, outline="black", fill="yellow")
        self.instruments_items.append(l)
        # Now draw the CDI moving part
        if cdi is not None:
            if cdi > 1:
                cdi = 1
            if cdi < -1:
                cdi = -1
            cdi_width_2 = 3 * hsi_size_2 / 4
            cdix = middle_width + cdi * cdi_width_2
            l = self.instruments.create_rectangle (cdix-2,move_top,
                    cdix+2,move_bottom, outline="black", fill="yellow")
            self.instruments_items.append(l)

        # Draw heading label
        r = self.instruments.create_rectangle (middle_width-20,middle_height-10,
                middle_width+20,middle_height+10, fill="yellow",
                outline="blue", width=3)
        self.instruments_items.append(r)
        label = self.instruments.create_text (middle_width,middle_height,
                text=str(heading), fill="black")
        self.instruments_items.append(label)

        # Turn rate indicator
        tr_y = 20
        tr_x = middle_width
        trwidth_2 = 60
        std_turn_deflection = 25.0
        std_turn_rate = 3.0     # degrees per sec
        c = self.instruments.create_oval (tr_x-5,tr_y-5, tr_x+5,tr_y+5,
                fill="black")
        self.instruments_items.append(c)
        hashmark = self.instruments.create_line (tr_x+trwidth_2+1,tr_y,
                    tr_x+trwidth_2+10,tr_y, width=3)
        self.instruments_items.append(hashmark)
        hashmark = self.instruments.create_line (tr_x-trwidth_2-1,tr_y,
                    tr_x-trwidth_2-10,tr_y, width=3)
        self.instruments_items.append(hashmark)
        hashmark = self.instruments.create_line (
                tr_x+(trwidth_2+1)*math.cos(std_turn_deflection * RAD_DEG),
                tr_y+(trwidth_2+1)*math.sin(std_turn_deflection * RAD_DEG),
                tr_x+(trwidth_2+10)*math.cos(std_turn_deflection * RAD_DEG),
                tr_y+(trwidth_2+10)*math.sin(std_turn_deflection * RAD_DEG), width=3)
        self.instruments_items.append(hashmark)
        hashmark = self.instruments.create_line (
                tr_x-(trwidth_2+1)*math.cos(std_turn_deflection * RAD_DEG),
                tr_y+(trwidth_2+1)*math.sin(std_turn_deflection * RAD_DEG),
                tr_x-(trwidth_2+10)*math.cos(std_turn_deflection * RAD_DEG),
                tr_y+(trwidth_2+10)*math.sin(std_turn_deflection * RAD_DEG), width=3)
        self.instruments_items.append(hashmark)
        trneedle_x1 = tr_x-(trwidth_2)*math.cos(turn_rate * std_turn_deflection/std_turn_rate * RAD_DEG)
        trneedle_x2 = tr_x+(trwidth_2)*math.cos(turn_rate * std_turn_deflection/std_turn_rate * RAD_DEG)
        trneedle_y1 = tr_y-(trwidth_2)*math.sin(turn_rate * std_turn_deflection/std_turn_rate * RAD_DEG)
        trneedle_y2 = tr_y+(trwidth_2)*math.sin(turn_rate * std_turn_deflection/std_turn_rate * RAD_DEG)
        needle = self.instruments.create_line (trneedle_x1,trneedle_y1,
                trneedle_x2,trneedle_y2, width=2)
        self.instruments_items.append(needle)

        # Display yaw
        yawy = self.instruments_height-20
        yawwidth_2 = 35
        yawheight_2 = 10
        pixels_per_yaw=2.0
        c = self.instruments.create_rectangle (middle_width-yawwidth_2, yawy-yawheight_2,
                middle_width+yawwidth_2, yawy+yawheight_2,
                fill="white", outline="black", width=2)
        yawx = middle_width+yaw*pixels_per_yaw
        bubblewidth_2 = 12
        bubbleheight_2 = 8
        c = self.instruments.create_oval (yawx-bubblewidth_2, yawy-bubbleheight_2,
                yawx+bubblewidth_2, yawy+bubbleheight_2,
                fill="white", outline="black", width=2)
        self.instruments_items.append(c)


    def display_runway(self):
        self.attitude.create_polygon (90,110, 110,110, 130,150, 70,150, fill="black")


def StartDisplayWindow():
    root = tk.Tk()
    display_window = TkDisplay(600, 450, master=root)
    return display_window

if __name__ == "__main__":
    display_window = StartDisplayWindow()
    display_window.update()
    pitch = 10
    pitch_increment = -1
    airspeed = 100
    altitude = 1010
    climb_rate = 500
    heading = 10
    cdi = 0.0
    turn_rate=-3        # degrees per second
    yaw = -5.0             # m/s^2 acceleration to the right
    while True:
        for roll in range (-10, 11, 1):
            display_window.display_horizon (pitch, roll)
            display_window.display_airspeed (airspeed)
            display_window.display_altitude (altitude)
            display_window.display_climb_rate (climb_rate)
            display_window.display_instruments (heading, cdi, turn_rate, yaw)
            altitude += 10
            display_window.update()
            time.sleep(.05)
            heading += 1
            if heading >= 360:
                heading = 0
            cdi += .03
            turn_rate += 6.0/20.0
            yaw += .1
        for roll in range (10, -11, -1):
            display_window.display_horizon (pitch, roll)
            display_window.display_airspeed (airspeed)
            display_window.display_altitude (altitude)
            display_window.display_climb_rate (climb_rate)
            display_window.display_instruments (heading, cdi, turn_rate, yaw)
            display_window.update()
            time.sleep(.05)
            airspeed += 1
            heading += 1
            if heading >= 360:
                heading = 0
            cdi -= .06
            turn_rate -= 6.0/20.0
            yaw -= .1
        cdi = 0
        climb_rate -= 100
        pitch += pitch_increment
        if pitch < -10:
            pitch_increment = 1
            pitch += pitch_increment
        elif pitch > 10:
            pitch_increment = -1
            pitch += pitch_increment
