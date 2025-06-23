#!/usr/bin/env python3
"""
tcp_server.py

ROS 1 TCP server to receive JSON messages (e.g., from Unity)
and forward them into the ROS system using `publish_from_unity`.

Key Features:
- Non-blocking select-based TCP server
- Accepts multiple clients
- Routes incoming messages to `AppRunner`
"""

import socket
import select
import logging


class TCPServer:
    def __init__(self, ros_node, logger=None, host='127.0.0.1', port=65432):
        """
        Args:
            ros_node: An object with a `publish_from_unity()` method (e.g., AppRunner).
            logger: Optional logger instance.
            host: Interface to bind (e.g., 0.0.0.0 for all).
            port: TCP port to listen on.
        """
        self.ros_node = ros_node
        self.logger = logger or logging.getLogger(__name__)
        self.host = host
        self.port = port

        self.server_socket = None
        self.sockets_list = []
        self.clients = {}
        self.running = False

    def start(self):
        """Starts the TCP server loop."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            self.server_socket.setblocking(False)

            self.sockets_list = [self.server_socket]
            self.running = True

            self.logger.info(f"[TCPServer] Listening on {self.host}:{self.port}")

            while self.running:
                read_sockets, _, exception_sockets = select.select(
                    self.sockets_list, [], self.sockets_list, 0.1
                )

                for notified_socket in read_sockets:
                    if notified_socket == self.server_socket:
                        self._accept_connection()
                    else:
                        self._receive_message(notified_socket)

                for sock in exception_sockets:
                    self._handle_exception(sock)

        except Exception as e:
            self.logger.exception(f"[TCPServer] Server error: {e}")
        finally:
            self.stop()

    def _accept_connection(self):
        client_socket, client_address = self.server_socket.accept()
        client_socket.setblocking(False)
        self.sockets_list.append(client_socket)
        self.clients[client_socket] = client_address
        self.logger.info(f"[TCPServer] New connection from {client_address}")

    def _receive_message(self, sock):
        try:
            message = sock.recv(4096)
            if not message:
                self._disconnect_client(sock)
                return

            decoded = message.decode()
            self.logger.debug(f"[TCPServer] Received: {decoded}")
            self.ros_node.publish_from_unity(decoded)

        except Exception as e:
            self.logger.error(f"[TCPServer] Error receiving from {self.clients.get(sock)}: {e}")
            self._disconnect_client(sock)

    def _disconnect_client(self, sock):
        addr = self.clients.get(sock, "Unknown")
        self.logger.info(f"[TCPServer] Disconnected: {addr}")
        if sock in self.sockets_list:
            self.sockets_list.remove(sock)
        self.clients.pop(sock, None)
        sock.close()

    def _handle_exception(self, sock):
        self.logger.warning(f"[TCPServer] Exception on socket {self.clients.get(sock)}")
        self._disconnect_client(sock)

    def stop(self):
        """Closes all sockets and stops the server."""
        self.running = False
        for sock in self.sockets_list:
            sock.close()
        self.logger.info("[TCPServer] Stopped.")
