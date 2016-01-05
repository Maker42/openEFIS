import logging, time
import util

logger=logging.getLogger(__name__)

class CmdMessenger:
    def __init__(self, port, arg_delimiter=',', cmd_delimiter=';', escape_char = '\\', binary=False):
        self._port = port
        self._arg_delimiter = arg_delimiter
        self._cmd_delimiter = cmd_delimiter
        self._escape_char = escape_char
        self._binary=binary
        if self._binary:
            raise RuntimeError ("Binary messaging unimplemented")
        self._sendcmd = ''
        self._command_handlers = dict()
        self._residual_recv = ''

    def sendCmdStart(self, cmdId):
        self._sendcmd = '%d'%cmdId

    def _find_escape_special(self, search_index):
        fa = self._sarg.find (self._arg_delimiter, search_index)
        fc = self._sarg.find (self._cmd_delimiter, search_index)
        fe = self._sarg.find (self._escape_char, search_index)
        ret = fa
        if fc >= 0 and (fc < ret or ret < 0):
            ret = fc
        if fe >= 0 and (fe < ret or ret < 0):
            ret = fe
        if ret >= 0:
            self._sarg = self._sarg[:ret] + self._escape_char + self._sarg[ret:]
            ret += 2
        return ret

    def sendCmdArg(self, a):
        self._sarg = str(a)
        search_index = self._find_escape_special (0)
        while search_index > 0:
            search_index = self._find_escape_special (search_index)
        self._sendcmd += self._arg_delimiter
        self._sendcmd += self._sarg

    def sendCmdEnd(self):
        logger.log(1, "Cmd Sending %s%s", self._sendcmd, self._cmd_delimiter)
        if self._port:
            self._port.write (self._sendcmd + self._cmd_delimiter + '\r\n')
        else:
            logger.warning("No port for command: %s"%self._sendcmd)

    def sendCmd(self, cmdId, *args):
        self.sendCmdStart(cmdId)
        for arg in args:
            self.sendCmdArg(arg)
        self.sendCmdEnd()

    def sendValidatedCmd(self, cmdId, *args):
        chksum = ComputeCheckSum (*args)
        allargs = tuple(args) + (chksum, )
        self.sendCmd (cmdId, *allargs)

    def attach(self, cmdId, fn):
        self._command_handlers[cmdId] = fn

    def _count_escapes(self, s, i):
        ret = 0
        while s[i] == self._escape_char:
            ret += 1
            i -= 1
            if i < 0:
                break
        return ret

    def _find_command_end(self, recv):
        cd = recv.find(self._cmd_delimiter)
        if cd == 0:
            return cd
        elif cd > 0 and (self._count_escapes (recv, cd) % 2) == 0:
            return cd
        else:
            return None

    def _rejoin_args(self, args):
        i = 0
        while i < len(args):
            if len (args[i]) > 0:
                if (self._count_escapes (args[i], len(args[i])-1) % 2) > 0:
                    assert(len(args) > i+1)
                    # Remove the escape, replace it with a delimiter and join with next arg
                    args[i] = args[i][:-1] + self._arg_delimiter + args[i+1]
                    del args[i+1]
                else:
                    i += 1
            else:
                i += 1

    def _remove_esc(self, a):
        ret = a
        index = ret.find(self._escape_char, 0)
        while index >= 0:
            ret = ret[:index] + ret[index+1:]
            index = ret.find(self._escape_char, index+1)
        return ret

    def _remove_escapes(self, args):
        ret = [self._remove_esc (a) for a in args]
        return ret

    def feedinSerialData (self):
        if self._port == None:
            # For testing
            # Sample 10DOF data
            line = '93.2,20.1,0,0,0,0.05,0.05,0.9,-0.8,-0.6,0.05,%d'%util.millis(time.time())
            chksum = ComputeCheckSum (line)
            self._command_handlers[3] ([line, chksum])
            # Sample GGA data
            line = '$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F'
            self._command_handlers[4] ([line])
            # Sample RMC data
            line = '$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A'
            self._command_handlers[4] ([line])
        else:
            rcv = self._port.read(1024)
            recv = self._residual_recv + rcv
            eoc = self._find_command_end (recv)
            while eoc != None:
                cmd = recv[:eoc]
                recv = recv[eoc+1:]
                if len(cmd) > 0:
                    logger.log(1, "Cmd Received %s", cmd)
                    args = cmd.split(self._arg_delimiter)
                    self._rejoin_args (args)
                    args = self._remove_escapes (args)
                    try:
                        cmdId = int(args[0])
                        if self._command_handlers.has_key(cmdId):
                            self._command_handlers[cmdId] (args[1:])
                        else:
                            print ("Invalid command %d received"%cmdId)
                    except Exception,e:
                        logger.warning("Error handling command %s: %s", str(args), str(e))
                eoc = self._find_command_end (recv)
            self._residual_recv = recv

def _checksum(chk, arg):
    if isinstance(arg,int):
        for shift in range(24,-8,-8):
            chk ^= ((arg >> shift) & 0xff)
    elif isinstance(arg,str):
        for c in arg:
            chk ^= ord(c)
    return int(chk)

def ComputeCheckSum(*args):
    ret = 0
    for arg in args:
        ret = _checksum(ret, arg)
    return ret
