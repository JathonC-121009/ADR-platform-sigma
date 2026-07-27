import socket
import json
import os

def main():
    # Setup UDP Socket to listen on localhost
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5050

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"[*] Listening for Gate Telemetry on {UDP_IP}:{UDP_PORT}...\n")

    try:
        while True:
            # Wait for data (1024 bytes is plenty for our small matrix)
            data, addr = sock.recvfrom(1024)
            
            # Decode the JSON payload
            payload = json.loads(data.decode('utf-8'))
            fps = payload.get("fps", 0)
            gates = payload.get("gates", [])

            # Clear the terminal so it looks like a live updating HUD
            # Note: use 'cls' instead of 'clear' if running on a Windows machine
            os.system('clear')

            print(f"=== LIVE DRONE TELEMETRY (Vision FPS: {fps}) ===")
            print("Format: [Dist(m), Fwd(X), Rgt(Y), Dwn(Z), Roll, Pitch, Yaw]\n")

            for i, gate in enumerate(gates):
                if gate[0] == 999.0:
                    print(f"  Gate {i+1}: --- NO DETECTION ---")
                else:
                    # Format numbers to look clean and aligned
                    formatted_gate = [f"{val:6.2f}" for val in gate]
                    print(f"  Gate {i+1}: [{', '.join(formatted_gate)}]")
                    
            print("\nPress CTRL+C to quit.")

    except KeyboardInterrupt:
        print("\n[*] Telemetry Reader Stopped.")

if __name__ == "__main__":
    main()
