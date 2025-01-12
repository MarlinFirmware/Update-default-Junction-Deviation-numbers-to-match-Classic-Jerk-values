#
# MarlinBinaryProtocol.py
# Supporting Firmware upload via USB/Serial, saving to the attached media.
#
import serial, math, time, threading, sys, datetime, random
from collections import deque

try:
    import heatshrink2 as heatshrink
    heatshrink_exists = True
except ImportError:
    try:
        import heatshrink
        heatshrink_exists = True
    except ImportError:
        heatshrink_exists = False

def millis():
    return time.perf_counter() * 1000

class TimeOut(object):
    def __init__(self, milliseconds):
        self.duration = milliseconds
        self.reset()

    def reset(self):
        self.endtime = millis() + self.duration

    def timedout(self):
        return millis() > self.endtime

class ReadTimeout(Exception):
    pass
class FatalError(Exception):
    pass
class SycronisationError(Exception):
    pass
class PayloadOverflow(Exception):
    pass
class ConnectionLost(Exception):
    pass

class Protocol(object):
    device = None
    baud = None
    max_block_size = 0
    port = None
    block_size = 0

    packet_transit = None
    packet_status = None
    packet_ping = None

    errors = 0
    packet_buffer = None
    simulate_errors = 0
    sync = 0
    connected = False
    syncronised = False
    worker_thread = None

    response_timeout = 1000

    applications = []
    responses = deque()

    def __init__(self, device, baud, bsize, simerr, timeout):
        print("pySerial Version:", serial.VERSION)
        self.port = serial.Serial(device, baudrate = baud, write_timeout = 0, timeout = 1)
        self.device = device
        self.baud = baud
        self.block_size = int(bsize)
        self.simulate_errors = max(min(simerr, 1.0), 0.0)
        self.connected = True
        self.response_timeout = timeout

        self.register(['ok', 'rs', 'ss', 'fe'], self.process_input)

        self.worker_thread = threading.Thread(target=Protocol.receive_worker, args=(self,))
        self.worker_thread.start()

    def receive_worker(self):
        while self.port.in_waiting:
            self.port.reset_input_buffer()

        def dispatch(data):
            for tokens, callback in self.applications:
                for token in tokens:
                    if token == data[:len(token)]:
                        callback((token, data[len(token):]))
                        return

        def reconnect():
            print("Reconnecting..")
            self.port.close()
            for x in range(10):
                try:
                    if self.connected:
                        self.port = serial.Serial(self.device, baudrate = self.baud, write_timeout = 0, timeout = 1)
                        return
                    else:
                        print("Connection closed")
                        return
                except:
                    time.sleep(1)
            raise ConnectionLost()

        while self.connected:
            try:
                data = self.port.readline().decode('utf8').rstrip()
                if len(data):
                    #print(data)
                    dispatch(data)
            except OSError:
                reconnect()
            except UnicodeDecodeError:
                # dodgy client output or datastream corruption
                self.port.reset_input_buffer()

    def shutdown(self):
        self.connected = False
        self.worker_thread.join()
        self.port.close()

    def process_input(self, data):
        #print(data)
        self.responses.append(data)

    def register(self, tokens, callback):
        self.applications.append((tokens, callback))

    def send(self, protocol, packet_type, data = bytearray()):
        self.packet_transit = self.build_packet(protocol, packet_type, data)
        self.packet_status = 0
        self.transmit_attempt = 0

        timeout = TimeOut(self.response_timeout * 20)
        while self.packet_status == 0:
            try:
                if timeout.timedout():
                    raise ConnectionLost()
                self.transmit_packet(self.packet_transit)
                self.await_response()
            except ReadTimeout:
                self.errors += 1
                #print("Packetloss detected..")
        self.packet_transit = None

    def await_response(self):
        timeout = TimeOut(self.response_timeout)
        while not len(self.responses):
            time.sleep(0.00001)
            if timeout.timedout():
                raise ReadTimeout()

        while len(self.responses):
            token, data = self.responses.popleft()
            switch = {'ok' : self.response_ok, 'rs': self.response_resend, 'ss' : self.response_stream_sync, 'fe' : self.response_fatal_error}
            switch[token](data)

    def send_ascii(self, data, send_and_forget = False):
        self.packet_transit = bytearray(data, "utf8") + b'\n'
        self.packet_status = 0
        self.transmit_attempt = 0

        timeout = TimeOut(self.response_timeout * 20)
        while self.packet_status == 0:
            try:
                if timeout.timedout():
                    return
                self.port.write(self.packet_transit)
                if send_and_forget:
                    self.packet_status = 1
                else:
                    self.await_response_ascii()
            except ReadTimeout:
                self.errors += 1
                #print("Packetloss detected..")
            except serial.SerialException:
                return
        self.packet_transit = None

    def await_response_ascii(self):
        timeout = TimeOut(self.response_timeout)
        while not len(self.responses):
            time.sleep(0.00001)
            if timeout.timedout():
                raise ReadTimeout()
        token, data = self.responses.popleft()
        self.packet_status = 1

    def corrupt_array(self, data):
        rid = random.randint(0, len(data) - 1)
        data[rid] ^= 0xAA
        return data

    def transmit_packet(self, packet):
        packet = bytearray(packet)
        if (self.simulate_errors > 0 and random.random() > (1.0 - self.simulate_errors)):
            if random.random() > 0.9:
                #random data drop
                start = random.randint(0, len(packet))
                end = start + random.randint(1, 10)
                packet = packet[:start] + packet[end:]
                #print("Dropping {0} bytes".format(end - start))
            else:
                #random corruption
                packet = self.corrupt_array(packet)
                #print("Single byte corruption")
        self.port.write(packet)
        self.transmit_attempt += 1

    def build_packet(self, protocol, packet_type, data = bytearray()):
        PACKET_TOKEN = 0xB5AD

        if len(data) > self.max_block_size:
            raise PayloadOverflow()

        packet_buffer = bytearray()

        packet_buffer += self.pack_int8(self.sync)                           # 8bit sync id
        packet_buffer += self.pack_int4_2(protocol, packet_type)             # 4 bit protocol id, 4 bit packet type
        packet_buffer += self.pack_int16(len(data))                          # 16bit packet length
        packet_buffer += self.pack_int16(self.build_checksum(packet_buffer)) # 16bit header checksum

        if len(data):
            packet_buffer += data
            packet_buffer += self.pack_int16(self.build_checksum(packet_buffer))

        packet_buffer =  self.pack_int16(PACKET_TOKEN) + packet_buffer       # 16bit start token, not included in checksum
        return packet_buffer

    # checksum 16 fletchers
    def checksum(self, cs, value):
        cs_low = (((cs & 0xFF) + value) % 255)
        return ((((cs >> 8) + cs_low) % 255) << 8) | cs_low

    def build_checksum(self, buffer):
        cs = 0
        for b in buffer:
            cs = self.checksum(cs, b)
        return cs

    def pack_int32(self, value):
        return value.to_bytes(4, byteorder='little')

    def pack_int16(self, value):
        return value.to_bytes(2, byteorder='little')

    def pack_int8(self, value):
        return value.to_bytes(1, byteorder='little')

    def pack_int4_2(self, vh, vl):
        value = ((vh & 0xF) << 4) | (vl & 0xF)
        return value.to_bytes(1, byteorder='little')

    def connect(self):
        print("Connecting: Switching Marlin to Binary Protocol...")
        self.send_ascii("M28B1")
        self.send(0, 1)

    def disconnect(self):
        self.send(0, 2)
        self.syncronised = False

    def response_ok(self, data):
        try:
            packet_id = int(data)
        except ValueError:
            return
        if packet_id != self.sync:
            raise SycronisationError()
        self.sync = (self.sync + 1) % 256
        self.packet_status = 1

    def response_resend(self, data):
        packet_id = int(data)
        self.errors += 1
        if not self.syncronised:
            print("Retrying syncronisation")
        elif packet_id != self.sync:
            raise SycronisationError()

    def response_stream_sync(self, data):
        sync, max_block_size, protocol_version = data.split(',')
        self.sync = int(sync)
        self.max_block_size = int(max_block_size)
        self.block_size = self.max_block_size if self.max_block_size < self.block_size else self.block_size
        self.protocol_version = protocol_version
        self.packet_status = 1
        self.syncronised = True
        print("Connection synced [{0}], binary protocol version {1}, {2} byte payload buffer".format(self.sync, self.protocol_version, self.max_block_size))

    def response_fatal_error(self, data):
        raise FatalError()


class FileTransferProtocol(object):
    protocol_id = 1

    class Packet(object):
        QUERY = 0
        OPEN  = 1
        CLOSE = 2
        WRITE = 3
        ABORT = 4

    responses = deque()
    def __init__(self, protocol, timeout = None):
        protocol.register(['PFT:success', 'PFT:version:', 'PFT:fail', 'PFT:busy', 'PFT:ioerror', 'PTF:invalid'], self.process_input)
        self.protocol = protocol
        self.response_timeout = timeout or protocol.response_timeout

    def process_input(self, data):
        #print(data)
        self.responses.append(data)

    def await_response(self, timeout = None):
        timeout = TimeOut(timeout or self.response_timeout)
        while not len(self.responses):
            time.sleep(0.0001)
            if timeout.timedout():
                raise ReadTimeout()

        return self.responses.popleft()

    def connect(self):
        self.protocol.send(FileTransferProtocol.protocol_id, FileTransferProtocol.Packet.QUERY)

        token, data = self.await_response()
        if token != 'PFT:version:':
            return False

        self.version, _, compression = data.split(':')
        if compression != 'none':
            algorithm, window, lookahead = compression.split(',')
            self.compression = {'algorithm': algorithm, 'window': int(window), 'lookahead': int(lookahead)}
        else:
            self.compression = {'algorithm': 'none'}

        print("File Transfer version: {0}, compression: {1}".format(self.version, self.compression['algorithm']))

    def open(self, filename, compression, dummy):
        payload =  b'\1' if dummy else b'\0'          # dummy transfer
        payload += b'\1' if compression else b'\0'    # payload compression
        payload += bytearray(filename, 'utf8') + b'\0'# target filename + null terminator

        timeout = TimeOut(5000)
        token = None
        self.protocol.send(FileTransferProtocol.protocol_id, FileTransferProtocol.Packet.OPEN, payload)
        while token != 'PFT:success' and not timeout.timedout():
            try:
                token, data = self.await_response(1000)
                if token == 'PFT:success':
                    print(filename,"opened")
                    return
                elif token == 'PFT:busy':
                    print("Broken transfer detected, purging")
                    self.abort()
                    time.sleep(0.1)
                    self.protocol.send(FileTransferProtocol.protocol_id, FileTransferProtocol.Packet.OPEN, payload)
                    timeout.reset()
                elif token == 'PFT:fail':
                    raise Exception("Can not open file on client")
            except ReadTimeout:
                pass
        raise ReadTimeout()

    def write(self, data):
        self.protocol.send(FileTransferProtocol.protocol_id, FileTransferProtocol.Packet.WRITE, data)

    def close(self):
        self.protocol.send(FileTransferProtocol.protocol_id, FileTransferProtocol.Packet.CLOSE)
        token, data = self.await_response(1000)
        if token == 'PFT:success':
            print("File closed")
            return True
        elif token == 'PFT:ioerror':
            print("Client storage device IO error")
            return False
        elif token == 'PFT:invalid':
            print("No open file")
            return False

    def abort(self):
        self.protocol.send(FileTransferProtocol.protocol_id, FileTransferProtocol.Packet.ABORT)
        token, data = self.await_response()
        if token == 'PFT:success':
            print("Transfer Aborted")

    def copy(self, filename, dest_filename, compression, dummy):
        self.connect()

        has_heatshrink = heatshrink_exists and self.compression['algorithm'] == 'heatshrink'
        if compression and not has_heatshrink:
            hs = '2' if sys.version_info[0] > 2 else ''
            print("Compression not supported by client. Use 'pip install heatshrink%s' to fix." % hs)
            compression = False

        data = open(filename, "rb").read()
        filesize = len(data)

        self.open(dest_filename, compression, dummy)

        block_size = self.protocol.block_size
        if compression:
            data = heatshrink.encode(data, window_sz2=self.compression['window'], lookahead_sz2=self.compression['lookahead'])

        cratio = filesize / len(data)

        blocks = math.floor((len(data) + block_size - 1) / block_size)
        kibs = 0
        dump_pctg = 0
        start_time = millis()
        for i in range(blocks):
            start = block_size * i
            end = start + block_size
            self.write(data[start:end])
            kibs = (( (i+1) * block_size) / 1024) / (millis() + 1 - start_time) * 1000
            if (i / blocks) >= dump_pctg:
                print("\r{0:2.0f}% {1:4.2f}KiB/s {2} Errors: {3}".format((i / blocks) * 100, kibs, "[{0:4.2f}KiB/s]".format(kibs * cratio) if compression else "", self.protocol.errors), end='')
                dump_pctg += 0.1
            if self.protocol.errors > 0:
                # Dump last status (errors may not be visible)
                print("\r{0:2.0f}% {1:4.2f}KiB/s {2} Errors: {3} - Aborting...".format((i / blocks) * 100, kibs, "[{0:4.2f}KiB/s]".format(kibs * cratio) if compression else "", self.protocol.errors), end='')
                print("")   # New line to break the transfer speed line
                self.close()
                print("Transfer aborted due to protocol errors")
                #raise Exception("Transfer aborted due to protocol errors")
                return False
        print("\r{0:2.0f}% {1:4.2f}KiB/s {2} Errors: {3}".format(100, kibs, "[{0:4.2f}KiB/s]".format(kibs * cratio) if compression else "", self.protocol.errors)) # no one likes transfers finishing at 99.8%

        if not self.close():
            print("Transfer failed")
            return False
        print("Transfer complete")
        return True


class EchoProtocol(object):
    def __init__(self, protocol):
        protocol.register(['echo:'], self.process_input)
        self.protocol = protocol

    def process_input(self, data):
        print(data)
