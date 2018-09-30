# Copyright (C) 2017-2018  Garrett Herschleb
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


import sys, os, logging, time

logger = logging.getLogger(__name__)

class ArduinoCmdMessenger:
    def __init__(self, messages):
        self.messages = messages
        self.quote = '/'
        self.arg_delim = '^'
        self.command_delim = ';'
        self.device = None
        self.receive_buffer = ''

    def StartComs(self, device_handle):
        self.device = device_handle
        time.sleep(2)

    def cmd_to_num(self, cmd):
        ret = self.messages.index(cmd)
        if ret >= 0:
            return ret
        else:
            raise RuntimeError ("Unknown command: %s"%cmd)

    def num_to_cmd(self, num):
        return self.messages[num]

    def crc(self, tosend):
        ret = 0
        for b in tosend:
            ret ^= ord(b)
        return str(ret)

    def sendCmd(self, cmd, args):
        return self.sendString (self.formatCmd (cmd, args))

    def formatCmd (self, cmd, args):
        tosend = str(self.cmd_to_num(cmd))
        for a in args:
            if type(a) == str:
                quoted = a.replace(self.quote, self.quote + self.quote)
                quoted = quoted.replace(self.arg_delim, self.quote + self.arg_delim)
                quoted = quoted.replace(self.command_delim, self.quote + self.command_delim)
                tosend += self.arg_delim + quoted
            else:
                tosend += self.arg_delim + str(a)
        tosend += self.arg_delim
        tosend += self.crc(tosend) + self.command_delim
        return tosend

    def sendString(self, tosend):
        logger.log (3, "%f, sending(%s): "%(time.time(), str(self.device.port)) + tosend)
        responses = list()
        good = False
        bad = False
        for retry in range(3):
            self.device.write(tosend.encode('ascii'))
            while True:
                recv = self.read_command()
                if recv is not None:
                    ack, rargs, tm = recv
                    if ack == "ack":
                        logger.log (3, "ack(%s)"%str(self.device.port))
                        responses.append(recv)
                        good = True
                        break
                    elif ack == "nack":
                        if rargs[0] == "crc":
                            logger.warning ("crc nack(%s)"%str(self.device.port))
                            break
                        else:
                            error = "nack: " + rargs[0]
                            logger.error (error)
                            bad = True
                            responses.append(recv)
                            break
                    else:
                        responses.append (recv)
                else:
                    logger.error ("No response from sense/control board (%s)"%str(self.device.port))
                    break
            if good or bad:
                break
        else:
            error = "retries exhausted"
            logger.error (error)
            if responses is not None:
                logger.error ("Other responses: %s", str(responses))
            raise RuntimeError (error)
        return responses

    def read_command(self, timeout=1.0):
        starttime = time.time()
        while True:
            recv = self.device.read()
            try:
                recv = recv.decode('utf-8')
            except:
                logger.debug ("cannot decode bytes: %s"%recv)
                recv = None
            if recv is not None:
                if len(recv) > 0:
                    self.receive_buffer += recv
                    eoc = self.command_delim_found (self.receive_buffer)
                    if eoc >= 0:
                        cmd,args = self.parse_recv(self.receive_buffer[:eoc])
                        self.receive_buffer = self.receive_buffer[eoc:]
                        if cmd is not None:
                            ts = time.time()
                            logger.log (3, "%f,receive(%s): %s %s"%(ts, str(self.device.port), cmd, str(args)))
                            return cmd,args,ts
            if time.time() >= starttime + timeout:
                break
        return None

    def command_delim_found(self, recv):
        potential_end = 0
        while (potential_end < len(recv)):
            if self.command_delim in recv[potential_end:]:
                potential_end = recv[potential_end:].index(self.command_delim)
                if potential_end > 0 and (recv[potential_end-1] != self.quote or
                        (potential_end > 1 and recv[potential_end-2] == self.quote)):
                    return potential_end + 1
                else:
                    potential_end += 1
            else:
                return -1
        return -1


    def parse_recv(self, recv):
        s = recv.split(self.arg_delim)
        a = 0
        # Handle quoted field seperators
        quoted_quote = self.quote + self.quote
        quoted_command_delim = self.quote + self.command_delim
        while a < len(s):
            if a+1 < len(s) and s[a].endswith(self.quote) and not s[a].endswith(quoted_quote):
                s[a] = s[a][:-1] + self.arg_delim + s[a+1]
                del s[a+1]
            else:
                a += 1

        # Handle quoted command delimeters
        a = 0
        while a < len(s):
            s[a] = s[a].replace(quoted_command_delim, self.command_delim)
            a += 1

        # Handle quoted quote characters
        a = 0
        while a < len(s):
            s[a] = s[a].replace(quoted_quote, self.quote)
            a += 1

        # Remove command delimiter
        if s[-1].endswith(self.command_delim):
            s[-1] = s[-1][:-1]

        # Interpret numeric fields
        a = 0
        while a < len(s):
            try:
                num = int(s[a])
                s[a] = num
            except:
                try:
                    num = float(s[a])
                    s[a] = num
                except:
                    pass
            a += 1

        # Replace command number with textual name and return
        cmd = s[0]
        args = s[1:]
        if type(cmd) == int:
            if cmd >= len(self.messages):
                return "Unknown",s
            else:
                return self.messages[cmd],args
        else:
            return "Invalid",s


    def recvStatus(self):
        return self.read_command()
