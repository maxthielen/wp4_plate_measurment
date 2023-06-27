import threading

from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer

FTP_PORT = 60000
FTP_USER = 'trispector'
FTP_PASSWORD = 'mecha'
FTP_DIR = '../../../data/png'

class TrispectorListener:
    def __init__(self) -> None:
        authorizer = DummyAuthorizer()
        authorizer.add_user(FTP_USER, FTP_PASSWORD, FTP_DIR, perm='elradfmw')

        handler = FTPHandler()
        handler.authorizer = authorizer
        handler.banner = "Flip Master 9000 FTP ready."

        address = ('10.0.3.249', FTP_PORT)
        server = FTPServer(address, handler)
        server.max_cons = 256
        server.max_cons_per_ip = 5

        self.server_thread = threading.Thread(target=server.serve_forever(), daemon=True)
        self.server_thread.start()