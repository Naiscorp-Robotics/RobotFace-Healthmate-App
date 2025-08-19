import socket
import threading

def handle_client(client_socket, addr):
    print(f"TCP Client connected from {addr}")
    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            print(f"TCP Received: {data.decode()}")
            # Echo back
            client_socket.send(data)
    except:
        pass
    finally:
        print(f"TCP Client {addr} disconnected")
        client_socket.close()

def start_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('localhost', 12345))
    server.listen(5)
    print("TCP Server started on localhost:12345")
    
    while True:
        client, addr = server.accept()
        client_handler = threading.Thread(target=handle_client, args=(client, addr))
        client_handler.start()

if __name__ == "__main__":
    start_tcp_server()
