import socket
import cv2
import numpy as np

def main():
    udp_ip = "0.0.0.0"  # Listen on all available interfaces
    udp_port = 54321

    # Create and bind the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    print(f"Listening on {udp_ip}:{udp_port}")

    while True:
        # Receive data from the UDP socket
        data, addr = sock.recvfrom(65535)  # Buffer size of 65535 bytes
        if data:
            # Convert the bytes to a NumPy array and decode as a JPEG image
            nparr = np.frombuffer(data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow("Received Image", img)
                # Press 'q' to quit the viewer
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to decode the received image.")

    sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
