# Copyright (C) 2015  Garrett Herschleb
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

import os
import sys

import socket, logging, asyncore
import argparse

import serial

if '__main__' == __name__:
    for p in sys.path:
        if 'Common' in p:
            break
    else:
        sys.path.append (os.path.join ('Common'))
        sys.path.append (os.path.join ('Test'))

import Globals

import util

command_queue = list()

class command_request_handler(asyncore.dispatcher_with_send):
    def __init__(self, sock, queue):
        self._command_queue = queue
        asyncore.dispatcher_with_send.__init__(self, sock)

    def handle_read(self):
        rec = self.recv(1024)
        if rec:
            rec = rec.strip()
            if rec:
                type_delimiter = rec.find(':')
                immediate = False
                if type_delimiter > 0:
                    cmds = rec.split(':', 1)
                    if len(cmds[0]) >= 1:
                        tp = cmds[0].strip()
                        if tp == 'I':
                            immediate = True
                        else:
                            immediate = False
                    command = cmds[1]
                else:
                    command = rec
                if command == 'reset':
                    self._command_queue = list()
                if immediate:
                    try:
                        print ("Executing command %s"%command)
                        eval("Globals.TheAircraft." + command)
                    except Exception,e:
                        logger.error ("Error (%s) executing command: %s", str(e), command)
                else:
                    self._command_queue.append(command)


class CommandServer(asyncore.dispatcher):
    def __init__(self, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind(("", port))
        self.listen(5)

    def initialize(self, filelines):
        self._command_queue = list()

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print 'Incoming connection from %s' % repr(addr)
            handler = command_request_handler(sock, self._command_queue)

    def GetNextDirective(self):
        ret = None
        if len(self._command_queue):
            ret = self._command_queue[0]
            del self._command_queue[0]
        return ret

if '__main__' == __name__:
    opt = argparse.ArgumentParser(description='Command control client')
    opt.add_argument('command', help='The command to send')
    #opt.add_argument('-s', '--server', action='store_true', help='Run a server instead (for debug)')
    opt.add_argument('-a', '--server-address', default='localhost', help='IP address of server')
    opt.add_argument('-p', '--server-port', default=48800, help = 'Port number of server')
    args = opt.parse_args()

    dist_path = '/usr/local/lib/python2.7/dist-packages'
    if os.path.isdir(dist_path):
        sys.path.append('/usr/local/lib/python2.7/dist-packages')

    #if args.server:
    #    s = CommandServer (args.server_port)
    #    asyncore.loop()
    #else:
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_sock.connect ((args.server_address, args.server_port))
    client_sock.send (args.command)
    client_sock.close()

