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

import time
import sys

import psycopg2

import Globals
from MicroServerComs import MicroServerComs

class EventDB(MicroServerComs):
    def __init__(self, dbname, user, table):
        MicroServerComs.__init__(self, "EventDB", input_mode='list')
        self.dbconn = psycopg2.connect ("dbname=%s user=%s"%(dbname, user))
        self.cur = self.dbconn.cursor()
        self.table = table

    def input(self, channel, input_fields, values):
        data = [str(round(v,3)) if isinstance(v,float) else str(v)
                    for infield,v in zip(input_fields,values)
                    if infield != 'timestamp']
        data_string = ','.join (data)
        if 'timestamp' in input_fields:
            #if not self.table_exists():
            #    self.create_table()
            tsindex = input_fields.index('timestamp')
            tm = values[tsindex]
            timestamp = time.strftime ('%Y-%m-%d %H:%M:%S.', time.gmtime(tm))
            seconds_fraction = tm - int(tm)
            timestamp += str(int(round(seconds_fraction * 1000)))   # Add milliseconds
            query = "INSERT INTO %s (timestamp, subject, verb, data) VALUES ('%s', '%s', '=', '%s');"%(
                    self.table, timestamp, channel, data_string)
        else:
            query = "INSERT INTO %s (timestamp, subject, verb, data) VALUES ('now', '%s', '=', '%s');"%(
                        self.table, channel, data_string)
        self.cur.execute (query)
        self.dbconn.commit()

    def table_exists(self):
        self.cur.execute ("select relname from pg_class where relname like '%s';"%self.table)
        try:
            if self.cur.fetchone():
                return True
        except:
            pass
        return False

    def create_table(self):
        with open("create_events_table.sql", "r") as scmd:
            query = scmd.readlines()
            scmd.close()
            self.cur.execute(query)

if __name__ == "__main__":
    eventdb = EventDB(sys.argv[1], sys.argv[2], sys.argv[3])
    eventdb.listen()
