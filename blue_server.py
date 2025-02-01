import bluetooth
import threading

# Define ESP32 MAC addresses
ESP32_MACS = [
    "10:06:1C:42:0F:12",  # Replace with actual MAC addresses
    "10:06:1C:82:44:7A",
]


def handle_client(mac_address):
    """Connects to the ESP32 device and receives data."""
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    try:
        sock.connect((mac_address, 1))  # RFCOMM channel 1
        print(f"Connected to {mac_address}")

        while True:
            data = sock.recv(1024)  # Receive data
            if not data:
                break
            print(f"Data from {mac_address}: {data.decode('utf-8')}")
    except Exception as e:
        print(f"Error with {mac_address}: {e}")
    finally:
        sock.close()
        print(f"Disconnected from {mac_address}")


def start_bluetooth_server():
    """Starts threads for each ESP32 connection."""
    threads = []
    for mac in ESP32_MACS:
        t = threading.Thread(target=handle_client, args=(mac,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()


if __name__ == "__main__":
    start_bluetooth_server()
