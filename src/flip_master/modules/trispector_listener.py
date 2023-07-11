import threading

from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer

FTP_PORT = 60000
FTP_USER = 'trispector'
FTP_PASSWORD = 'mecha'
FTP_DIR = '.'

class TrispectorListener:
    def __init__(self) -> None:
        print('starting')
        authorizer = DummyAuthorizer()
        authorizer.add_user(FTP_USER, FTP_PASSWORD, FTP_DIR, perm='elradfmwMT')
        print('handler')
        handler = FTPHandler
        handler.authorizer = authorizer
        handler.banner = "Flip Master 9000 FTP ready."
        print('server')
        address = ('192.168.1.130', FTP_PORT)
        print('jh')
        server = FTPServer(address, handler)
        print('here')
        server.max_cons = 256
        server.max_cons_per_ip = 5
        # server.serve_forever()
        # print('thread')
        self.server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        # print('threading')
        self.server_thread.start()