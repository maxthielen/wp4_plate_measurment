import threading

from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer

FTP_PORT = 21
FTP_USER = 'trispector'
FTP_PASSWORD = 'mecha'
FTP_DIR = '/home/ros_ws'

class TrispectorListener:
    def __init__(self) -> None:
        authorizer = DummyAuthorizer()
        authorizer.add_user(FTP_USER, FTP_PASSWORD, FTP_DIR, perm='elradfmwMT')
        
        handler = FTPHandler
        handler.authorizer = authorizer
        handler.banner = "Flip Master 9000 FTP ready."

        address = ('localhost', FTP_PORT)
        server = FTPServer(address, handler)
        server.max_cons = 256
        server.max_cons_per_ip = 5

        self.server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        self.server_thread.start()