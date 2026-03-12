import socket
import threading
import json
import queue
import time

class Server: # Robot Server
    def __init__(self):
        self.host = "0.0.0.0"
        self.port = 8080
        # AF_INET is used for IPv4 protocols
        # SOCK_STREAM is used for TCP packets
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        print("[Server] Socket binded to %s:%d" % (self.host, self.port))
        self.server_socket.listen(1) # Allow only 1 connection
        self.client_socket = None
        self.input_queue = queue.Queue() # For incoming commands and ACKs
        self.output_queue = queue.Queue() # For outgoing telemetry and ACKs
        self.cmd_queue = queue.Queue() # Queue to send incoming commands
        self.running = True
        self.pending_acks = {}
        self.ack_timeout = 0.5 # seconds
        self.message_id = -1

    def start(self):
        print("[Server] Waiting for client...")
        self.client_socket, addr = self.server_socket.accept()
        print("[Server] Got connection from", addr)

        threading.Thread(target=self._receiver_thread).start()
        threading.Thread(target=self._sender_thread).start()
        threading.Thread(target=self._ack_monitor_thread).start()

        while self.running:
            try:
                msg = self.input_queue.get(timeout=1)  # Wait up to 1 second for a message
                if msg.get('type') == 'command':
                    self.cmd_queue.put(msg)
                    # Send ACK for received command
                    ack = {"type": "ack", "id": msg.get('id')}
                    self.output_queue.put(ack)
                elif msg.get('type') == 'ack':
                    # Handle ACK: Remove from pending
                    msg_id = msg.get('id')
                    if msg_id in self.pending_acks:
                        del self.pending_acks[msg_id]
            except queue.Empty:
                continue

    def get_command(self):
        try:
            msg = self.cmd_queue.get_nowait()  # Get the full message
            print(f"[Server] Sent to robot: {msg}")
            return msg.get('data')  # Return only the 'data' part
        except queue.Empty:
            return None  # No command available

    def _receiver_thread(self):
        stream = self.client_socket.makefile('r')
        try:
            while self.running:
                raw = stream.readline() # Read a line from the stream
                if not raw: # If no data is read (connection closed)
                    print("[Server] Connection closed by controller.")
                    self.stop()
                    break
                raw = raw.strip()
                if not raw: # If the string is empty
                    continue
                msg = json.loads(raw)
                self.input_queue.put(msg)
                #print(f"[Server] Received: {msg}")
        finally:
            stream.close()

    def send_telemetry(self, data):
        msg = {"type": "telemetry", "id": self.message_id, "data": data}
        self.output_queue.put(msg)
        self.pending_acks[self.message_id] = (msg, time.time())
        self.message_id -= 1


    def _sender_thread(self):
        stream = self.client_socket.makefile('w')
        try:
            while self.running:
                try:
                    msg = self.output_queue.get(timeout=1) # Wait up to 1 second for a message
                    json_str = json.dumps(msg) + '\n'
                    stream.write(json_str)
                    stream.flush() # Flush the stream to ensure data is sent immediately
                    #print(f"[Server] Sent: {msg}")
                except queue.Empty:
                    continue
        finally:
            stream.close()

    def _ack_monitor_thread(self):
        while self.running:
            current_time = time.time()
            to_resend = []
            for msg_id, (msg, sent_time) in self.pending_acks.items():
                if current_time - sent_time > self.ack_timeout:
                    to_resend.append(msg)
            for msg in to_resend:
                print(f"[Server] Resending unacknowledged message: {msg}")
                self.output_queue.put(msg)
                self.pending_acks[msg['id']] = (msg, time.time()) # Update the timestamp in pending ACKs
            time.sleep(0.5)  # Check every 0.5 seconds

    def stop(self):
        self.running = False
        try:
            self.server_socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.server_socket.close()
