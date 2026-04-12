# save as server.py
from http.server import BaseHTTPRequestHandler, HTTPServer
import cgi

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        ctype, pdict = cgi.parse_header(self.headers['Content-Type'])
        
        if ctype == 'multipart/form-data':
            pdict['boundary'] = bytes(pdict['boundary'], "utf-8")
            pdict['CONTENT-LENGTH'] = int(self.headers['Content-Length'])
            
            fields = cgi.parse_multipart(self.rfile, pdict)
            file_data = fields['file'][0]
            
            with open("received_image.jpg", "wb") as f:
                f.write(file_data)
            
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"File received")
        else:
            self.send_response(400)
            self.end_headers()

server = HTTPServer(('0.0.0.0', 8000), SimpleHTTPRequestHandler)
print("Server running on port 8000...")
server.serve_forever()