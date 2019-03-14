# Copyright (C) 2018-2019  Garrett Herschleb
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

import math, threading

import yaml

import Common.util as util

from MicroServerComs import MicroServerComs

class AirspeedComputed(MicroServerComs):
    def __init__(self, airspeed_config):
        MicroServerComs.__init__(self, "AirspeedComputed")
        self.pitot = None
        self.airspeed_computed = None
        self.intercept = None
        self.A = None
        self.in_curve_fitting = False
        if isinstance(airspeed_config,dict):
            self.rais_id = airspeed_config['rais_id']
            if 'intercept' in airspeed_config:
                self.intercept = airspeed_config['intercept']
                self.A = airspeed_config['A']
            else:
                self.compute_curve()

    def updated(self, channel):
        if channel == 'pitotsensor':
            if self.A is not None:
                # Only output if we have 2 valid pressures to compare
                self.timestamp = self.pitotsensor_updated
                self.airspeed_computed = compute_airspeed (self.pitot,
                                    self.intercept, self.A)
                self.publish ()
                print ("AirspeedComputed: %.2f ==> %.2f"%(self.pitot,
                            self.airspeed_computed))
            else:
                print ("AirspeedComputed: don't have curve")
        elif channel == 'admincommand' and self.pitot is not None:
            if self.command.startswith(b'airspeed'):
                if not self.in_curve_fitting:
                    given_file = 'given_airspeed%d.csv'%self.rais_id
                    f = open(given_file, 'a+')
                    f.write ('%.2f,%.1f\n'%(self.pitot, float(self.args.strip(b'\x00'))))
                    f.close()
                    self.compute_curve()
        elif channel == 'systemcommand':
            if self.command.startswith(b'0air'):
                print ("Airspeed: Received command to 0 out")
                self.intercept = self.pitot

    def compute_curve(self):
        if self.in_curve_fitting:
            return
        self.in_curve_fitting = True
        self.fit_thread = threading.Thread(target=self.fit_curve)
        self.fit_thread.start()

    def fit_curve(self):
        given_file = 'given_airspeed%d.csv'%self.rais_id
        f = open(given_file, 'r')
        points = list()         # (pitot reading, IAS)
        while True:
            l = f.readline()
            if len(l) == 0:
                break
            l = l.strip()
            if len(l) == 0:
                continue
            points.append([float(x) for x in l.split(',')])
        f.close()
        if len(points):
            # Estimate intercept
            p0 = [p for p in points if p[1] == 0]
            self.intercept = float(sum([p[0] for p in p0])) / len(p0)
            # Estimate initial A
            # pitot pressure = A * v^2
            # A =  pitot pressure / v^2 
            # pitot_pressure = reading - intercept
            # A =  (reading - intercept) / v^2 
            maxv = 0
            maxreading = 0
            for reading,v in points:
                if v > maxv:
                    maxv = v
                    maxreading = reading
            A = (maxreading - self.intercept) / (maxv*maxv)

            # Now find best A
            start_A = A
            best_A = self.search_rds (points, start_A, start_A, .001)
            best_A = self.search_rds (points, start_A, best_A, -.001)
            self.A = best_A
        print ("************** New airspeed curve. Intercept = %.2f, A=%.4f"%(
            self.intercept, self.A))
        self.in_curve_fitting = False

    def search_rds(self, points, start_A, best_A, increment):
        minrds = compute_rds (points, self.intercept, start_A)
        A = start_A + increment
        n_losses = 0
        while True:
            rds = compute_rds (points, self.intercept, A)
            if rds > minrds:
                n_losses += 1
                if n_losses > 20:
                    break
            elif minrds > rds:
                n_losses = 0
                minrds = rds
                best_A = A
            A += increment
        return best_A


def compute_rds(points, intercept, A):
    ret = 0
    for reading,v in points:
        y = compute_airspeed (reading, intercept, A)
        diff = y - v
        ret += diff*diff
    return math.sqrt(ret)

def compute_airspeed (reading, intercept, A):
    # pitot pressure = A * v^2
    # v^2 = pitot pressure / A
    # v = sqrt(pitot pressure / A)
    # pitot_pressure = reading - intercept
    pp = (reading - intercept)
    if pp < 0:
        pp = 0
    v = math.sqrt(pp / A)
    return v

if __name__ == "__main__":
    ae = AirspeedComputed()
    ae.listen()
